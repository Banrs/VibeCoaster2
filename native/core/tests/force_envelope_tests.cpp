#include "coaster/force_envelope.hpp"
#include <complex>
#include <iostream>
#include <stdexcept>
using namespace coaster;
static int checks;
static void check(bool ok,const char* why){++checks;if(!ok)throw std::runtime_error(why);}
static bool near(double a,double b,double tolerance=1e-8){return std::abs(a-b)<=tolerance;}
using Histories=std::array<std::vector<double>,3>;
static Histories history(double duration,double step,const std::function<SeatForces(double)>& values){
    Histories out;const size_t count=static_cast<size_t>(std::llround(duration/step))+1;
    for(size_t i=0;i<count;++i){auto f=values(i*step);out[0].push_back(f.vertical);out[1].push_back(f.lateral);out[2].push_back(f.longitudinal);}return out;
}
static ForceEnvelopeAssessment held(int axis,double value,double duration,double step=1./960){
    return assessForceEnvelope(history(duration,step,[&](double){SeatForces f{};if(axis==0)f.vertical=value;if(axis==1)f.lateral=value;if(axis==2)f.longitudinal=value;return f;}),step);
}
static bool has(const ForceEnvelopeAssessment& result,const char* code){return std::any_of(result.report.errors.begin(),result.report.errors.end(),[&](const Finding& f){return f.code==code;});}
static double pulse(double t,double begin,double end,double ramp){return smooth((t-begin)/ramp)*smooth((end-t)/ramp);}
// Independent test oracle: multiply the four digital pole factors and run one
// fourth-order direct recurrence, rather than the production two-biquad filter.
static std::vector<double> referenceFilter(const std::vector<double>& input,double step){
    using Complex=std::complex<double>;std::array<Complex,5> a{};a[0]=1;Complex gain=1;const double k=std::tan(pi*5*step);
    for(int pole=0;pole<4;++pole){Complex analog=std::polar(1.,pi*(2*pole+5)/8),digital=(1.+k*analog)/(1.-k*analog);gain*=1.-digital;
        for(int j=pole+1;j>0;--j)a[j]-=digital*a[j-1];}
    constexpr double numerator[5]={1,4,6,4,1};std::vector<double> output(input.size());
    for(size_t i=0;i<input.size();++i){double y=0;for(int j=0;j<5;++j)y+=gain.real()/16*numerator[j]*(i>=size_t(j)?input[i-j]:input.front());
        for(int j=1;j<5;++j)y-=a[j].real()*(i>=size_t(j)?output[i-j]:input.front());output[i]=y;}return output;
}
static double referenceExcursion(const std::vector<double>& values,double step,ForceAxis axis,bool positive){
    double worst=0,maximum=0;for(double v:values)maximum=std::max(maximum,positive?v:-v);
    // Dense independent crossing scan. Its grid lower-bounds the exact all-level
    // maximum; it is intentionally unrelated to the production stack algorithm.
    for(double level=.003;level<maximum;level+=.003){double begin=0;bool active=(positive?values[0]:-values[0])>=level;
        auto assess=[&](double end){double duration=end-begin;if(duration>=.2)worst=std::max(worst,level/historicalForceLimit(axis,positive,duration));};
        for(size_t i=1;i<values.size();++i){double a=positive?values[i-1]:-values[i-1],b=positive?values[i]:-values[i];
            if((a>=level)!=(b>=level)){double crossing=(i-1+(level-a)/(b-a))*step;if(active)assess(crossing);else begin=crossing;active=!active;}}
        if(active)assess((values.size()-1)*step);}
    return worst;
}
int main(){try{
    for(double step:{1./960,1./1920}){
        auto steady=held(0,1,5,step);check(steady.performed&&steady.report.valid(),"Steady 1 g is valid");
        check(near(steady.axes[0].minimumG,1,1e-10)&&near(steady.axes[0].maximumG,1,1e-10),"Filter starts in the actual steady seat state without a startup force transient");
        check(std::abs(steady.axes[0].minimumOnsetGps)<1e-7&&std::abs(steady.axes[0].maximumOnsetGps)<1e-7,"Steady initialized history has no invented onset");
        check(held(0,6,.8,step).report.valid(),"Full +6 g range is usable for a short sustained event");
        check(has(held(0,6,1.25,step),"FORCE_DURATION"),"+6 g cannot be held past the declining duration boundary");
        check(held(0,4,3,step).report.valid()&&has(held(0,4.01,3,step),"FORCE_DURATION"),"The 2-4 second +4 g shelf is enforced at its boundary");
        check(held(0,3,8,step).report.valid()&&has(held(0,3.01,8,step),"FORCE_DURATION"),"A long sustained positive event is checked against its own +3 g shelf");
        check(held(1,3,.8,step).report.valid()&&has(held(1,3,2,step),"FORCE_DURATION"),"Both lateral peak and sustained ranges are supported");
        check(held(1,-3,.8,step).report.valid()&&has(held(1,-3,2,step),"FORCE_DURATION"),"Signed lateral excursions are symmetric");
        check(held(2,6,.8,step).report.valid()&&has(held(2,5,2,step),"FORCE_DURATION"),"Longitudinal launch force uses its duration curve");
        check(held(2,2.5,15,step).report.valid()&&has(held(2,2.6,15,step),"FORCE_DURATION"),"Longitudinal positive long-duration limit is 2.5 g, not the vertical 2 g limit");
        check(held(0,-2.8,.2,step).report.valid()&&has(held(0,-2.8,.5,step),"FORCE_DURATION"),"Selected extended negative-Gz range reaches -2.8 g at 200 ms and tapers immediately with duration");
        check(held(0,-2.2,1,step).report.valid()&&has(held(0,-2.21,1,step),"FORCE_DURATION"),"Extended negative-Gz one-second boundary is -2.2 g");
        check(held(0,-1.5,4,step).report.valid()&&has(held(0,-1.5,7,step),"FORCE_DURATION"),"Extended negative-Gz returns to the normal sustained shelf and long-duration limit");
        check(held(2,-3.5,1.8,step).report.valid()&&has(held(2,-3.5,2.5,step),"FORCE_DURATION"),"Padded-OTS exception reaches -3.5 g with low onset, but cannot hold it indefinitely");
        check(held(2,-2,10,step).report.valid()&&has(held(2,-2.1,10,step),"FORCE_DURATION"),"Higher longitudinal braking curve tends to -2 g, not the positive +6 g magnitude");
        auto braking=[&](double ramp,double peak){return assessForceEnvelope(history(4,step,[&](double t){return SeatForces{0,0,-peak*smooth((t-1)/ramp)};}),step);};
        auto hardBrake=braking(.12,3),controlledBrake=braking(.8,3),ordinaryBrake=braking(.06,1.5);
        check(has(hardBrake,"FORCE_ENHANCED_LONGITUDINAL_ONSET")&&!has(controlledBrake,"FORCE_ENHANCED_LONGITUDINAL_ONSET"),"Higher negative-Gx allowance requires actual negative-load buildup below 15 g/s");
        check(!has(ordinaryBrake,"FORCE_ENHANCED_LONGITUDINAL_ONSET"),"Ordinary braking within 2 g is not subjected to an invented blanket onset limit");
        auto earlyBuildup=assessForceEnvelope(history(4,step,[](double t){return SeatForces{0,0,-1.7*smooth((t-1)/.08)-smooth((t-1.6)/.8)};}),step);
        check(has(earlyBuildup,"FORCE_ENHANCED_LONGITUDINAL_ONSET")&&earlyBuildup.enhancedLongitudinalOnset.startSeconds<1.6,"Enhanced braking qualification includes a rapid initial buildup before the force reaches 2 g");
        auto release=assessForceEnvelope(history(3,step,[](double t){return SeatForces{0,0,-2.5*smooth((t-.5)/.8)*smooth((2-t)/.05)};}),step);
        check(release.axes[2].maximumOnsetGps>15&&!has(release,"FORCE_ENHANCED_LONGITUDINAL_ONSET"),"An actually rapid load release is not subjected to a fabricated absolute-rate limit");
        auto mixedBrake=[&](double ramp){return assessForceEnvelope(history(2.2,step,[&](double t){return SeatForces{4.8*pulse(t,.5,1.9,.2),0,-1.5*smooth((t-.7)/ramp)};}),step);};
        check(has(mixedBrake(.05),"FORCE_PAIRED")&&!has(mixedBrake(.8),"FORCE_PAIRED"),"Combined loading cannot borrow the higher negative-Gx ellipse radius when its onset condition is unqualified");
        auto combined=assessForceEnvelope(history(.6,step,[](double){return SeatForces{5.5,1.8,0};}),step);
        check(!has(combined,"FORCE_DURATION")&&has(combined,"FORCE_PAIRED"),"Simultaneous individually permitted lateral/normal loads must pass the quadrant ellipse too");
        check(near(combined.paired[0].actual,5.5*5.5/36+1.8*1.8/9,1e-9),"Pairwise result uses actual signed 200 ms ellipse radii");
        auto transient=assessForceEnvelope(history(.15,step,[](double){return SeatForces{6,3,6};}),step);
        check(!has(transient,"FORCE_PAIRED"),"An ellipse excursion shorter than 200 ms is not relabelled sustained; raw impact guards remain caller-owned");
        auto shortBuildup=assessForceEnvelope(history(2,step,[](double t){return SeatForces{0,0,-3.4*smooth((t-1.8)/.1)};}),step);
        check(shortBuildup.axes[2].minimumG>-3.5&&shortBuildup.axes[2].minimumG<-3.4&&shortBuildup.axes[2].minimumOnsetGps<-30,
            "A short terminal negative-X event stays inside the enhanced peak range but has independently excessive measured buildup");
        check(has(shortBuildup,"FORCE_ENHANCED_LONGITUDINAL_ONSET"),
            "A negative-X event shorter than 200 ms cannot borrow the higher restraint exception without qualifying its onset");
        auto embedded=assessForceEnvelope(history(20,step,[](double t){return SeatForces{1+4.2*pulse(t,8,9,.25),0,0};}),step);
        check(!has(embedded,"FORCE_DURATION"),"A short positive peak inside a long positive baseline is not assigned the baseline's duration limit");
        auto longPeak=assessForceEnvelope(history(20,step,[](double t){return SeatForces{1+3.6*pulse(t,8,12,.25),0,0};}),step);
        check(has(longPeak,"FORCE_DURATION"),"A long high-force subevent cannot hide inside a benign whole-run mean");
        auto rise=[&](double ramp,double peak){return assessForceEnvelope(history(4,step,[&](double t){return SeatForces{-.8+(peak+.8)*std::clamp((t-2)/ramp,0.,1.),0,0};}),step);};
        auto fast=rise(.2,3),slow=rise(.7,3),low=rise(.4,1.6);
        check(has(fast,"FORCE_ZERO_TO_TWO")&&!has(slow,"FORCE_ZERO_TO_TWO"),"Sustained nonpositive-to-positive transition checks actual interpolated 0-to-2 g duration");
        check(fast.zeroToTwo.actual<.133&&slow.zeroToTwo.actual>.133,"Transition evidence reports elapsed seconds, not a guessed instantaneous derivative");
        check(low.zeroToTwo.utilization==0,"A rise which never reaches 2 g does not trigger the 0-to-2 g clause");
        auto uplift=[&](double begin){return assessForceEnvelope(history(14,step,[&](double t){double base=-.5+1.5*smooth((t-4)/1.);return SeatForces{base+4.2*pulse(t,begin,begin+1,.25),0,0};}),step);};
        auto during=uplift(8),after=uplift(11.5);
        check(has(during,"FORCE_POST_UPLIFT")&&!has(after,"FORCE_POST_UPLIFT"),"Reduced positive limit applies in the verified six-second window and normal limits resume afterwards");
        check(during.reducedPositiveFromSeconds>4&&during.reducedPositiveFromSeconds<5,"Post-uplift window begins at the actual transition to positive, not when negative exposure first reaches three seconds");
        auto blipInput=history(7,step,[](double t){return SeatForces{-.4+1.4*smooth((t-3.4)/.5)+pulse(t,2.7,2.85,.03)+4.2*pulse(t,4.5,5.5,.25),0,0};});
        auto blip=assessForceEnvelope(blipInput,step);auto blipFiltered=referenceFilter(blipInput[0],step);double blipSeconds=0;
        for(size_t i=1;i<blipFiltered.size();++i)if(i*step>2.5&&i*step<3.2&&blipFiltered[i]>0)blipSeconds+=step;
        check(blipSeconds>.03&&blipSeconds<.2&&has(blip,"FORCE_POST_UPLIFT"),"An actual filtered positive blip shorter than 200 ms cannot reset the explicit project negative-history policy");
        auto reversal=[&](double scale,double separation){return assessForceEnvelope(history(4,step,[&](double t){double a=(t-(2-separation*.5))/.055,b=(t-(2+separation*.5))/.055;return SeatForces{0,(t<2?.3:-.3)+scale*(std::exp(-a*a)-std::exp(-b*b)),0};}),step);};
        auto rapid=reversal(2.5,.16),gentle=reversal(.5,.16),spaced=reversal(2.5,.4);
        check(has(rapid,"FORCE_HORIZONTAL_REVERSAL")&&!has(gentle,"FORCE_HORIZONTAL_REVERSAL")&&!has(spaced,"FORCE_HORIZONTAL_REVERSAL"),"Only rapid sufficiently large reversals between sustained horizontal events invoke the half-limit rule");
        auto conditionalReversal=assessForceEnvelope(history(4,step,[](double t){double a=(t-1.92)/.055,b=(t-2.08)/.055;return SeatForces{0,0,(t<2?.2:-.2)+2*std::exp(-a*a)-1.6*std::exp(-b*b)};}),step);
        check(conditionalReversal.axes[2].minimumG>-2&&conditionalReversal.axes[2].minimumOnsetGps<-15&&has(conditionalReversal,"FORCE_HORIZONTAL_REVERSAL")&&near(conditionalReversal.horizontalReversal[1].limit,1),"An onset-unqualified negative event uses the ordinary OTS 1 g reversal half-limit, not the higher exception's 1.75 g");
    }
    auto nearBoundaryReversal=[](double t){
        const double a=(t-1.973+.21*.5)/.055,b=(t-1.973-.21*.5)/.055;
        return SeatForces{1,0,(t<1.973?.2:-.2)+.3*(std::exp(-a*a)-std::exp(-b*b))};
    };
    const auto referenceReversal=assessForceEnvelope(history(4,1./15360,nearBoundaryReversal),1./15360).horizontalReversal[1];
    check(referenceReversal.utilization>.27&&referenceReversal.durationSeconds<.2,"A densely sampled continuous signal independently resolves the near-200 ms reversal");
    for(double step:{1./960,1./1920}){
        const auto reversal=assessForceEnvelope(history(4,step,nearBoundaryReversal),step).horizontalReversal[1];
        check(near(reversal.utilization,referenceReversal.utilization,.001)&&near(reversal.durationSeconds,referenceReversal.durationSeconds,step*.5),"Isolated peak timing resolves the same near-boundary event within half a sample at both mandatory rates");
        auto ramp=history(3,step,[](double t){return SeatForces{.4+1.5*t,0,-.2-t};});auto measured=assessForceEnvelope(ramp,step);
        for(size_t axis:std::array<size_t,2>{0,2}){auto filtered=referenceFilter(ramp[axis],step);const size_t half=size_t(std::llround(.05/step));double minimum=0,maximum=0;
            for(size_t center=half;center+half<filtered.size();++center){double numerator=0,denominator=0;for(int offset=-int(half);offset<=int(half);++offset){double time=offset*step;numerator+=time*filtered[center+offset];denominator+=time*time;}double slope=numerator/denominator;minimum=std::min(minimum,slope);maximum=std::max(maximum,slope);}
            check(near(measured.axes[axis].minimumOnsetGps,minimum,2e-5)&&near(measured.axes[axis].maximumOnsetGps,maximum,2e-5),"Centered 100 ms onset agrees with independent direct least-squares windows on a filtered linear ramp");}
        auto sine=assessForceEnvelope(history(10,step,[](double t){return SeatForces{0,smooth(t/3)*std::sin(2*pi*5*t),0};}),step);
        check(near(sine.axes[1].maximumG,1/std::sqrt(2.),.002),"Fourth-order single-pass 5 Hz cutoff has the expected -3 dB gain");
        check(std::abs(std::remainder(sine.axes[1].maximumOnsetTimeSeconds-.1,.2))<.004,"Cutoff response retains the causal single-pass phase, rather than a zero-phase double filter");
    }
    auto varied=history(12,1./960,[](double t){return SeatForces{1.8+2.7*std::sin(.73*t)+.9*std::sin(2.4*t),0,0};});auto variedAssessment=assessForceEnvelope(varied,1./960);auto independent=referenceFilter(varied[0],1./960);
    for(size_t sign=0;sign<2;++sign){double oracle=referenceExcursion(independent,1./960,ForceAxis::Vertical,sign==0),actual=variedAssessment.directional[sign].utilization;
        check(actual+2e-6>=oracle&&actual-oracle<.004,"All-level excursion result brackets an independent dense isoacceleration crossing oracle");}
    auto convergenceInput=[](double step){return history(15,step,[](double t){return SeatForces{1+3.8*pulse(t,2,4.8,.6)-2.5*pulse(t,6,7,.35),.9*std::sin(1.2*t),-2.5*pulse(t,10,12,.9)};});};
    auto coarse=assessForceEnvelope(convergenceInput(1./960),1./960),fine=assessForceEnvelope(convergenceInput(1./1920),1./1920);
    for(size_t i=0;i<6;++i)check(std::abs(coarse.directional[i].utilization-fine.directional[i].utilization)<.005,"Signed excursion utilization converges at the mandatory rates");
    for(size_t i=0;i<3;++i){check(std::abs(coarse.paired[i].utilization-fine.paired[i].utilization)<.005,"Pairwise sustained utilization converges at the mandatory rates");
        check(std::abs(coarse.axes[i].maximumOnsetGps-fine.axes[i].maximumOnsetGps)<.02,"Centered onset converges at the mandatory rates");}
    check(held(0,1,200).report.valid(),"The positive 2 g/40 s endpoint does not reject normal gravity throughout a ride");
    check(held(0,2,39).report.valid()&&has(held(0,2,41),"FORCE_DURATION_EXTENT"),"Positive 2 g duration does not silently extend past 40 seconds");
    check(has(held(1,1,91),"FORCE_DURATION_EXTENT"),"The lateral published 90-second domain remains explicit");
    for(double step:std::array<double,5>{0.,-.01,.1,NAN,INFINITY})check(has(assessForceEnvelope(history(1,1./960,[](double){return SeatForces{1,0,0};}),step),"FORCE_ENVELOPE_INPUT"),"Invalid sample step cannot be assessed");
    auto invalid=history(1,1./960,[](double){return SeatForces{1,0,0};});invalid[1].pop_back();check(has(assessForceEnvelope(invalid,1./960),"FORCE_ENVELOPE_INPUT"),"Different axis lengths cannot silently truncate a seat history");
    invalid=history(1,1./960,[](double){return SeatForces{1,0,0};});invalid[2][17]=NAN;check(has(assessForceEnvelope(invalid,1./960),"FORCE_ENVELOPE_INPUT"),"Nonfinite forces fail closed");
    auto longHistory=history(60,1./1920,[](double t){return SeatForces{1+.4*std::sin(t),.2*std::cos(t),0};});int calls=0;
    auto cancelled=assessForceEnvelope(longHistory,1./1920,[&]{return ++calls==40;});check(cancelled.cancelled&&!cancelled.performed,"Cancellation interrupts force processing and cannot look like completed validation");
    auto cancellationInput=history(6,1./960,[](double t){return SeatForces{1+.2*std::sin(t),.1,0};});
    calls=0;assessForceEnvelope(cancellationInput,1./960,[&]{++calls;return false;});
    for(int cancelAt=1;cancelAt<=calls;++cancelAt){int current=0;auto once=assessForceEnvelope(cancellationInput,1./960,[&]{return ++current==cancelAt;});
        check(once.cancelled&&!once.performed,"Every single cancellation response is retained, including sign-event processing");}
    std::cout<<"PASS "<<checks<<" historical signed force-envelope checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<checks<<": "<<e.what()<<'\n';return 1;}}
