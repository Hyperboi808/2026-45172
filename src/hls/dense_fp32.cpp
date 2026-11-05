// Eight output lanes; exact eight-stripe FP32 accumulation; burst output.
// Original FP32 weights stay in PS DDR. Register ABI is unchanged.
#include <ap_int.h>
static const int MAX_INPUT=288, MAX_OUTPUT=256, MAX_ELEMENTS=8192, LANES=8;
extern "C" void dense_fp32(float *memory, unsigned input_offset,
    unsigned weight_offset, unsigned bias_offset, unsigned output_offset,
    unsigned batch, unsigned inputs, unsigned outputs, unsigned relu) {
#pragma HLS INTERFACE m_axi port=memory offset=slave bundle=memory depth=200000 max_read_burst_length=64 max_write_burst_length=64 num_read_outstanding=4 num_write_outstanding=4
#pragma HLS INTERFACE s_axilite port=memory bundle=control
#pragma HLS INTERFACE s_axilite port=input_offset bundle=control
#pragma HLS INTERFACE s_axilite port=weight_offset bundle=control
#pragma HLS INTERFACE s_axilite port=bias_offset bundle=control
#pragma HLS INTERFACE s_axilite port=output_offset bundle=control
#pragma HLS INTERFACE s_axilite port=batch bundle=control
#pragma HLS INTERFACE s_axilite port=inputs bundle=control
#pragma HLS INTERFACE s_axilite port=outputs bundle=control
#pragma HLS INTERFACE s_axilite port=relu bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control
    if (!batch || !inputs || !outputs || inputs>MAX_INPUT || outputs>MAX_OUTPUT ||
        batch>256 || batch*inputs>MAX_ELEMENTS || batch*outputs>MAX_ELEMENTS) return;
    float x[MAX_ELEMENTS], y[MAX_ELEMENTS], w[LANES][MAX_INPUT], bias[LANES];
#pragma HLS BIND_STORAGE variable=x type=ram_1p impl=bram
#pragma HLS BIND_STORAGE variable=y type=ram_1p impl=bram
#pragma HLS ARRAY_PARTITION variable=w complete dim=1
#pragma HLS ARRAY_PARTITION variable=bias complete
load_inputs:
    for(unsigned k=0;k<batch*inputs;k++) {
#pragma HLS PIPELINE II=1
        x[k]=memory[input_offset+k];
    }
output_tiles:
    for(unsigned row=0;row<outputs;row+=LANES) {
load_lanes:
        for(unsigned lane=0;lane<LANES;lane++) {
            bias[lane]=row+lane<outputs?memory[bias_offset+row+lane]:0.f;
load_weights:
            for(unsigned k=0;k<inputs;k++) {
#pragma HLS PIPELINE II=1
                w[lane][k]=row+lane<outputs?memory[weight_offset+(row+lane)*inputs+k]:0.f;
            }
        }
samples:
        for(unsigned sample=0;sample<batch;sample++) {
            float sums[LANES][8], totals[LANES];
#pragma HLS ARRAY_PARTITION variable=sums complete dim=0
#pragma HLS ARRAY_PARTITION variable=totals complete
init_sums:
            for(unsigned lane=0;lane<LANES;lane++) {
#pragma HLS UNROLL
                totals[lane]=bias[lane];
                for(unsigned a=0;a<8;a++) {
#pragma HLS UNROLL
                    sums[lane][a]=0.f;
                }
            }
dot_products:
            for(unsigned k=0;k<inputs;k++) {
#pragma HLS PIPELINE II=1
                float value=x[sample*inputs+k];
                for(unsigned lane=0;lane<LANES;lane++) {
#pragma HLS UNROLL
                    float product=value*w[lane][k];
#pragma HLS BIND_OP variable=product op=fmul impl=fulldsp latency=4
                    float next=sums[lane][0]+product;
                    // Rotate a fixed-index register queue: each stripe returns
                    // to the adder after eight iterations, without alias muxes.
                    for(unsigned a=0;a<7;a++) {
#pragma HLS UNROLL
                        sums[lane][a]=sums[lane][a+1];
                    }
                    sums[lane][7]=next;
                }
            }
// Keep bias + stripe0 + ... + stripe7 in exactly the original order.
reduce_stripes:
            for(unsigned a=0;a<8;a++) {
#pragma HLS PIPELINE off
                for(unsigned lane=0;lane<LANES;lane++) {
#pragma HLS UNROLL
                    totals[lane]=totals[lane]+sums[lane][(a+8-(inputs&7))&7];
                }
            }
buffer_results:
            for(unsigned lane=0;lane<LANES;lane++) {
#pragma HLS PIPELINE II=1
                float sum=totals[lane];
                if(relu && sum<0.f) sum=0.f;
                if(row+lane<outputs) y[sample*outputs+row+lane]=sum;
            }
        }
    }
store_results:
    for(unsigned k=0;k<batch*outputs;k++) {
#pragma HLS PIPELINE II=1
        memory[output_offset+k]=y[k];
    }
}
