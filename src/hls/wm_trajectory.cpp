// Actual frozen WM action head: checkpoint table gather and exact radian decode.
// Route selection stays on PS. No dormant neural head is used as a substitute.
#include <ap_int.h>
#include <stdint.h>
#include "wm_calibration.h"
static const unsigned MAX_COUNT=128;
static unsigned round_even(double value) {
    unsigned base=(unsigned)value;
    double fraction=value-(double)base;
    return base+((fraction>0.5 || (fraction==0.5 && (base&1)))?1:0);
}
extern "C" unsigned wm_trajectory(uint32_t *memory,unsigned reference_offset,
    unsigned request_offset,unsigned output_offset,unsigned count) {
#pragma HLS INTERFACE m_axi port=memory offset=slave bundle=memory depth=300000 max_read_burst_length=64 max_write_burst_length=64 num_read_outstanding=4 num_write_outstanding=4
#pragma HLS INTERFACE s_axilite port=memory bundle=control
#pragma HLS INTERFACE s_axilite port=reference_offset bundle=control
#pragma HLS INTERFACE s_axilite port=request_offset bundle=control
#pragma HLS INTERFACE s_axilite port=output_offset bundle=control
#pragma HLS INTERFACE s_axilite port=count bundle=control
#pragma HLS INTERFACE s_axilite port=return bundle=control
    if(!count || count>MAX_COUNT)return 1;
    uint32_t requests[MAX_COUNT],output[MAX_COUNT*14*3];
#pragma HLS BIND_STORAGE variable=output type=ram_1p impl=bram
load_requests:
    for(unsigned i=0;i<count;i++) {
#pragma HLS PIPELINE II=1
        requests[i]=memory[request_offset+i];
    }
    unsigned invalid=0;
validate_requests:
    for(unsigned i=0;i<count;i++) {
#pragma HLS PIPELINE II=1
        unsigned r=requests[i];
        invalid|=((r>>16)>=20 || ((r>>8)&255)>=7 || (r&255)>=60)?1:0;
    }
    if(invalid)return 2;
waypoints:
    for(unsigned i=0;i<count;i++) {
        unsigned r=requests[i],route=r>>16,phase=(r>>8)&255,tick=r&255;
        unsigned source=reference_offset+((route*7+phase)*60+tick)*14;
joint_decode:
        for(unsigned j=0;j<14;j++) {
#pragma HLS PIPELINE off
            union {uint32_t u;float f;} raw;
            raw.u=memory[source+j];
            double x=(double)raw.f;
            if(!(x==x))return 3;
            if(x < -1.0)x=-1.0;
            if(x > 1.0)x=1.0;
            unsigned step;
            if(j==2) {
                unsigned travel=round_even((x+1.0)*2048.0);
                if(travel>4095)travel=4095;
                step=(WRIST_HOME+travel)&4095;
            } else step=round_even((double)LOW[j]+(x+1.0)*(double)(HIGH[j]-LOW[j])/2.0);
            if(step<LOW[j])step=LOW[j];
            if(step>HIGH[j])step=HIGH[j];
            union {uint64_t u;double d;} rad;
            rad.d=((double)step-OFFSET[j])/SCALE[j];
            unsigned dest=(i*14+j)*3;
            output[dest]=step;
            output[dest+1]=(uint32_t)rad.u;
            output[dest+2]=(uint32_t)(rad.u>>32);
        }
    }
burst_output:
    for(unsigned i=0;i<count*42;i++) {
#pragma HLS PIPELINE II=1
        memory[output_offset+i]=output[i];
    }
    return 0;
}
