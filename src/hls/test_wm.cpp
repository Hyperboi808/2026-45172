#include <cstdio>
#include <vector>
#include <cstdint>
#include <cmath>
#include <algorithm>
extern "C" unsigned wm_trajectory(uint32_t*,unsigned,unsigned,unsigned,unsigned);
int main() {
    FILE*f=fopen("wm_cases.bin","rb");if(!f)return 2;
    unsigned words,cases;
    if(fread(&words,4,1,f)!=1 || fread(&cases,4,1,f)!=1)return 3;
    std::vector<uint32_t> original(words),memory(300000,0xaabbccdd);
    if(fread(original.data(),4,words,f)!=words)return 4;
    unsigned req=words,out=req+128;
    double worst=0;unsigned values=0;
    for(unsigned test=0;test<cases;test++) {
        unsigned count,patch;
        if(fread(&count,4,1,f)!=1 || fread(&patch,4,1,f)!=1)return 5;
        std::fill(memory.begin(),memory.end(),0xaabbccdd);
        std::copy(original.begin(),original.end(),memory.begin());
        if(patch && fread(memory.data(),4,14,f)!=14)return 6;
        if(fread(memory.data()+req,4,count,f)!=count)return 7;
        std::vector<uint32_t> expected(count*42);
        if(fread(expected.data(),4,count*42,f)!=count*42)return 8;
        std::vector<uint32_t> before(memory.begin(),memory.begin()+out);
        unsigned error=wm_trajectory(memory.data(),0,req,out,count);
        if(error){printf("FAIL status=%u test=%u\n",error,test);return 9;}
        for(unsigned i=0;i<count*14;i++) {
            if(memory[out+i*3]!=expected[i*3])return 10;
            union{uint64_t u;double d;} actual,want;
            actual.u=(uint64_t(memory[out+i*3+2])<<32)|memory[out+i*3+1];
            want.u=(uint64_t(expected[i*3+2])<<32)|expected[i*3+1];
            double delta=std::fabs(actual.d-want.d);worst=std::max(worst,delta);values++;
            if(!std::isfinite(actual.d) || delta>1e-12)return 11;
        }
        for(unsigned i=0;i<out;i++)if(before[i]!=memory[i])return 12;
        for(unsigned i=out+count*42;i<memory.size();i++)if(memory[i]!=0xaabbccdd)return 13;
        printf("PASS WM case=%u waypoints=%u\n",test,count);
    }
    std::fill(memory.begin(),memory.end(),0xaabbccdd);
    if(wm_trajectory(memory.data(),0,0,100,0)!=1)return 14;
    if(wm_trajectory(memory.data(),0,0,100,129)!=1)return 15;
    memory[0]=20<<16;
    if(wm_trajectory(memory.data(),0,0,100,1)!=2)return 16;
    memory[0]=7<<8;
    if(wm_trajectory(memory.data(),0,0,100,1)!=2)return 17;
    memory[0]=60;
    if(wm_trajectory(memory.data(),0,0,100,1)!=2)return 18;
    for(unsigned i=100;i<memory.size();i++)if(memory[i]!=0xaabbccdd)return 19;
    fclose(f);printf("WM_TEST_PASS cases=%u joint_values=%u max_rad_error=%.15g\n",cases,values,worst);return 0;
}
