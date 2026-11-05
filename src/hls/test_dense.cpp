#include <cstdio>
#include <vector>
#include <cmath>
#include <algorithm>
extern "C" void dense_fp32(float*,unsigned,unsigned,unsigned,unsigned,unsigned,unsigned,unsigned,unsigned);
int main() {
    FILE *f=fopen("dense_cases.bin","rb"); if(!f) return 2;
    unsigned count=0;if(fread(&count,4,1,f)!=1)return 3;
    double worst=0; unsigned values=0;
    for(unsigned test=0;test<count;test++) {
        unsigned h[4];if(fread(h,4,4,f)!=4)return 4;
        unsigned b=h[0],n=h[1],o=h[2],r=h[3];
        unsigned iw=b*n,wo=iw,bo=wo+n*o,oo=bo+o;
        std::vector<float> mem(200000,-12345.f),expected(b*o);
        if(fread(&mem[0],4,oo,f)!=oo || fread(&expected[0],4,b*o,f)!=b*o)return 5;
        dense_fp32(&mem[0],0,wo,bo,oo,b,n,o,r);
        for(unsigned k=0;k<b*o;k++) {
            double error=std::fabs(double(mem[oo+k])-expected[k]);
            worst=std::max(worst,error);values++;
            if(!std::isfinite(mem[oo+k]) || error>3e-4+3e-5*std::fabs(expected[k])) {
                printf("FAIL case=%u index=%u got=%g expected=%g\n",test,k,mem[oo+k],expected[k]);return 6;
            }
        }
        for(unsigned k=oo+b*o;k<mem.size();k++)if(mem[k]!=-12345.f)return 7;
        printf("PASS case=%u batch=%u in=%u out=%u\n",test,b,n,o);
    }
    fclose(f);
    // Invalid shapes must return without touching memory.
    std::vector<float> sentinel(200000,0);
    for(unsigned k=0;k<8;k++)sentinel[k]=k+1;
    dense_fp32(&sentinel[0],0,0,0,0,1,289,2,0);
    dense_fp32(&sentinel[0],0,0,0,0,257,3,2,0);
    dense_fp32(&sentinel[0],0,0,0,0,0,3,2,0);
    for(unsigned k=0;k<8;k++)if(sentinel[k]!=float(k+1))return 8;
    printf("DENSE_TEST_PASS cases=%u values=%u max_abs_error=%.9g\n",count,values,worst);return 0;
}
