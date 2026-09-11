#include "coaster/force_envelope.hpp"
#include "force_envelope_parameters.hpp"
#include <span>

namespace coaster {
namespace {
struct Point {double time,limit;};
constexpr Point verticalPositive[]={{.2,6},{1,6},{2,4},{4,4},{5,3},{11.8,3},{12,2}};
constexpr Point verticalNegative[]={{.2,2.8},{1,2.2},{3,1.5},{4,1.5},{7,1.1}};
constexpr Point lateral[]={{.2,3},{1,3},{2,2}};
constexpr Point longitudinalPositive[]={{.2,6},{1,6},{2,4},{4,4},{5,3},{11.8,3},{12,2.5}};
constexpr Point longitudinalNegative[]={{.2,3.5},{2,3.5},{3,2.5},{4,2.5},{5,2}};
constexpr Point reducedPositive[]={{.2,5},{1.5,5},{2,4},{2.5,2}};
std::span<const Point> curve(ForceAxis axis,bool positive,bool reduced=false){
    if(axis==ForceAxis::Vertical)return positive?(reduced?std::span<const Point>(reducedPositive):std::span<const Point>(verticalPositive)):std::span<const Point>(verticalNegative);
    if(axis==ForceAxis::Lateral)return lateral;
    return positive?std::span<const Point>(longitudinalPositive):std::span<const Point>(longitudinalNegative);
}
double limitAt(std::span<const Point> points,double time){
    if(time<=points.front().time)return points.front().limit;
    for(size_t i=1;i<points.size();++i)if(time<=points[i].time){const auto a=points[i-1],b=points[i];return a.limit+(b.limit-a.limit)*(time-a.time)/(b.time-a.time);}
    return points.back().limit;
}
void worst(ForceEnvelopeCase& out,double actual,double limit,double start,double duration,bool inverse=false){
    const double utilization=inverse?limit/actual:actual/limit;
    if(utilization>out.utilization)out={utilization,actual,limit,start,duration};
}
bool stopped(const Cancel& cancel,size_t i){return (i&4095)==0&&cancel&&cancel();}

std::vector<double> filter(const std::vector<double>& input,double step,const Cancel& cancel){
    // Bilinear-transform fourth-order Butterworth, prewarped at 5 Hz.
    struct Section {double b0,b1,b2,a1,a2,z1,z2;double apply(double x){double y=b0*x+z1;z1=b1*x-a1*y+z2;z2=b2*x-a2*y;return y;}};
    std::array<Section,2> sections;
    const double k=std::tan(pi*5*step);
    for(size_t i=0;i<2;++i){const double q=i?1.3065629648763766:.541196100146197;const double n=1/(1+k/q+k*k),b=k*k*n,a2=(1-k/q+k*k)*n;
        sections[i]={b,2*b,b,2*(k*k-1)*n,a2,(1-b)*input.front(),(b-a2)*input.front()};}
    std::vector<double> output(input.size());output[0]=input.front();
    for(size_t i=1;i<input.size();++i){if(stopped(cancel,i))return {};double y=input[i];for(auto& section:sections)y=section.apply(y);output[i]=y;}
    return output;
}

// A monotone nearest-smaller stack finds every connected superlevel excursion.
// Between adjacent height levels its two interpolated crossing times are affine
// in height. Utilization is therefore monotone between duration-curve vertices:
// endpoints and those vertices suffice; a peak sample alone does not.
bool excursions(const std::vector<double>& signedValues,bool positive,double step,
    std::span<const Point> points,ForceEnvelopeCase& out,const Cancel& cancel,
    ForceEnvelopeCase* extent=nullptr,double maximumDuration=0,double durationThreshold=0){
    const size_t count=signedValues.size();std::vector<double> values(count);
    std::vector<size_t> left(count),right(count),stack;stack.reserve(count);
    for(size_t i=0;i<count;++i){if(stopped(cancel,i))return false;values[i]=std::max(0.,positive?signedValues[i]:-signedValues[i]);
        while(!stack.empty()&&values[stack.back()]>=values[i])stack.pop_back();left[i]=stack.empty()?count:stack.back();stack.push_back(i);}
    stack.clear();
    for(size_t i=count;i-->0;){if(stopped(cancel,i))return false;while(!stack.empty()&&values[stack.back()]>=values[i])stack.pop_back();right[i]=stack.empty()?count:stack.back();stack.push_back(i);}
    for(size_t i=0;i<count;++i){if(stopped(cancel,i))return false;const double high=values[i];if(high<=0)continue;
        const size_t l=left[i],r=right[i];const double low=std::max(l==count?0:values[l],r==count?0:values[r]);
        auto times=[&](double height){
            double begin=l==count?0:(l+(height-values[l])/(values[l+1]-values[l]))*step;
            double end=r==count?(count-1)*step:(r-1+(values[r-1]-height)/(values[r-1]-values[r]))*step;
            return std::array<double,2>{begin,end-begin};};
        const auto highTimes=times(high),lowTimes=times(low);
        auto assess=[&](double height){const auto td=times(height);if(td[1]+1e-12<.2)return;worst(out,height,limitAt(points,td[1]),td[0],td[1]);
            if(extent&&height>=durationThreshold)worst(*extent,td[1],maximumDuration,td[0],td[1]);};
        assess(high);if(low>0)assess(low);
        if(lowTimes[1]>highTimes[1])for(const auto p:points)if(p.time>=highTimes[1]&&p.time<=lowTimes[1])assess(high+(low-high)*(p.time-highTimes[1])/(lowTimes[1]-highTimes[1]));
        if(extent&&durationThreshold>low&&durationThreshold<high)assess(durationThreshold);
    }
    return true;
}

struct Event {int sign;double begin,end,peak,firstPeak,lastPeak;};
std::vector<Event> events(const std::vector<double>& values,double step,const Cancel& cancel){
    std::vector<Event> result;
    auto append=[&](int sign,double begin,double end,double a,double b){if(end<=begin)return;
        if(result.empty()||result.back().sign!=sign)result.push_back({sign,begin,end,0,begin,begin});
        auto& event=result.back();event.end=end;
        for(auto p:std::array<std::pair<double,double>,2>{{{std::abs(a),begin},{std::abs(b),end}}}){
            if(p.first>event.peak+1e-10){event.peak=p.first;event.firstPeak=event.lastPeak=p.second;}
            else if(std::abs(p.first-event.peak)<=1e-10)event.lastPeak=p.second;}
    };
    for(size_t i=1;i<values.size();++i){if(stopped(cancel,i))return {};double a=values[i-1],b=values[i],begin=(i-1)*step,end=i*step;
        if(a*b<0){double crossing=begin+step*std::abs(a)/(std::abs(a)+std::abs(b));append(a>0?1:-1,begin,crossing,a,0);append(b>0?1:-1,crossing,end,0,b);}
        else append((a>0||b>0)?1:(a<0||b<0)?-1:0,begin,end,a,b);}
    // A strict sampled peak has a local quadratic time estimate. Using only
    // grid positions can put the same smooth reversal on opposite sides of
    // 200 ms at the mandatory rates. Keep sampled magnitudes and the existing
    // first/last extents of flat peaks unchanged.
    for(auto& event:result)if(event.firstPeak==event.lastPeak){
        const size_t i=static_cast<size_t>(std::llround(event.firstPeak/step));
        if(i==0||i+1>=values.size())continue;
        const double a=event.sign*values[i-1],b=event.sign*values[i],c=event.sign*values[i+1];
        if(a<b&&c<b)event.firstPeak=event.lastPeak=(i+.5*(a-c)/(a-2*b+c))*step;
    }
    return result;
}

bool onset(const std::vector<double>& values,double step,ForceEnvelopeAxisSummary& summary,const Cancel& cancel,std::vector<double>* slopes=nullptr){
    if((values.size()-1)*step<.1)return true;
    const size_t radius=static_cast<size_t>(std::llround(.05/step)),width=2*radius+1;
    if(radius==0||values.size()<width)return true;
    double sum=0,moment=0;for(size_t i=0;i<width;++i){sum+=values[i];moment+=(double(i)-radius)*values[i];}
    const double divisor=step*double(width)*double(width*width-1)/12;
    for(size_t center=radius;center+radius<values.size();++center){if(stopped(cancel,center))return false;
        const double slope=moment/divisor;
        if(slopes)(*slopes)[center]=slope;
        if(slope<summary.minimumOnsetGps){summary.minimumOnsetGps=slope;summary.minimumOnsetTimeSeconds=center*step;}
        if(slope>summary.maximumOnsetGps){summary.maximumOnsetGps=slope;summary.maximumOnsetTimeSeconds=center*step;}
        if(center+radius+1<values.size()){double old=values[center-radius],next=values[center+radius+1];moment+=-sum+(radius+1)*old+radius*next;sum+=next-old;}}
    return true;
}

void reportCase(ValidationReport& report,const ForceEnvelopeCase& value,const char* code,const char* message){
    if(value.utilization>1+1e-9)report.fail(code,message,value.startSeconds,value.actual,value.limit);
}
}

double historicalForceLimit(ForceAxis axis,bool positive,double durationSeconds,bool afterUplift){return limitAt(curve(axis,positive,afterUplift),durationSeconds);}

ForceEnvelopeAssessment assessForceEnvelope(const std::array<std::vector<double>,3>& input,double step,Cancel cancel){
    ForceEnvelopeAssessment result;
    const size_t count=input[0].size();
    if(!std::isfinite(step)||step<=0||step>=.1||count<2||input[1].size()!=count||input[2].size()!=count){result.report.fail("FORCE_ENVELOPE_INPUT","Force envelope requires equal finite uniformly sampled z/y/x histories, sample rate above 10 Hz, and at least two samples");return result;}
    std::array<std::vector<double>,3> filtered;
    std::vector<double> longitudinalOnset(count,0);
    std::vector<bool> enhancedLongitudinal(count,true);
    for(size_t axis=0;axis<3;++axis){for(size_t i=0;i<count;++i){if(stopped(cancel,i)){result.cancelled=true;return result;}if(!std::isfinite(input[axis][i])){result.report.fail("FORCE_ENVELOPE_INPUT","Nonfinite seat force cannot be assessed",i*step);return result;}}
        filtered[axis]=filter(input[axis],step,cancel);if(filtered[axis].empty()){result.cancelled=true;return result;}
        const auto bounds=std::minmax_element(filtered[axis].begin(),filtered[axis].end());result.axes[axis].minimumG=*bounds.first;result.axes[axis].maximumG=*bounds.second;
        if(!onset(filtered[axis],step,result.axes[axis],cancel,axis==2?&longitudinalOnset:nullptr)){result.cancelled=true;return result;}
        for(size_t sign=0;sign<2;++sign)if(!excursions(filtered[axis],sign==0,step,curve(static_cast<ForceAxis>(axis),sign==0),result.directional[2*axis+sign],cancel,axis==1?&result.durationExtent[1]:axis==0&&sign==0?&result.durationExtent[0]:nullptr,axis==1?90:40,axis==0?2:1e-9)){result.cancelled=true;return result;}
    }
    double negativeDuration=0,nonpositiveDuration=0;
    std::vector<std::array<double,2>> reducedWindows;
    const auto verticalEvents=events(filtered[0],step,cancel);if(verticalEvents.empty()){result.cancelled=true;return result;}
    for(const auto& event:verticalEvents){const double duration=event.end-event.begin;
        if(event.sign<0)negativeDuration+=duration;
        else if(event.sign==0){if(duration>=.2)negativeDuration=0;}
        if(event.sign<=0){nonpositiveDuration+=duration;continue;}
        if(negativeDuration>=3){if(std::isinf(result.reducedPositiveFromSeconds))result.reducedPositiveFromSeconds=event.begin;
            if(!reducedWindows.empty()&&event.begin<=reducedWindows.back()[1])reducedWindows.back()[1]=event.begin+6;
            else reducedWindows.push_back({event.begin,event.begin+6});}
        if(nonpositiveDuration>=.2){
            size_t first=std::max<size_t>(1,static_cast<size_t>(std::ceil(event.begin/step))),last=std::min(count-1,static_cast<size_t>(std::floor(event.end/step)));
            bool crossed=false;
            for(size_t i=first;i<=last;++i){if(stopped(cancel,i)){result.cancelled=true;return result;}
                if(!crossed&&filtered[0][i]>=2&&filtered[0][i-1]<2){double t=(i-1)*step+step*(2-filtered[0][i-1])/(filtered[0][i]-filtered[0][i-1]);worst(result.zeroToTwo,t-event.begin,detail::zeroToTwoMinimumSeconds,event.begin,t-event.begin,true);crossed=true;}
            }
        }
        if(duration>=.2){negativeDuration=0;nonpositiveDuration=0;}
    }
    if(!reducedWindows.empty()){
        std::vector<double> reduced(count);size_t window=0;
        for(size_t i=0;i<count;++i){if(stopped(cancel,i)){result.cancelled=true;return result;}while(window<reducedWindows.size()&&i*step>=reducedWindows[window][1])++window;
            if(window<reducedWindows.size()&&i*step>=reducedWindows[window][0])reduced[i]=filtered[0][i];}
        if(!excursions(reduced,true,step,reducedPositive,result.reducedPositive,cancel)){result.cancelled=true;return result;}
    }
    auto radiusAt=[&](size_t axis,bool positive,size_t sample){return axis==2&&!positive&&!enhancedLongitudinal[sample]?2:historicalForceLimit(static_cast<ForceAxis>(axis),positive,.2);};
    for(size_t axis=1;axis<3;++axis){auto signedEvents=events(filtered[axis],step,cancel);if(signedEvents.empty()){result.cancelled=true;return result;}const Event* previous=nullptr;
        for(const auto& event:signedEvents){if(event.sign==0)continue;
            if(axis==2&&event.sign<0){
                const size_t first=static_cast<size_t>(std::ceil(event.begin/step)),last=std::min(count-1,static_cast<size_t>(std::floor(event.end/step)));
                ForceEnvelopeCase buildup;
                for(size_t i=first;i<=last;++i){if(stopped(cancel,i)){result.cancelled=true;return result;}worst(buildup,std::max(0.,-longitudinalOnset[i]),15,i*step,.1);}
                if(event.peak>2&&buildup.utilization>result.enhancedLongitudinalOnset.utilization)result.enhancedLongitudinalOnset=buildup;
                if(buildup.utilization>=1)std::fill(enhancedLongitudinal.begin()+first,enhancedLongitudinal.begin()+last+1,false);}
            if(event.end-event.begin<.2)continue; // Duration excludes sustained reversals, not restraint-exception qualification.
            if(previous&&previous->sign!=event.sign&&event.firstPeak-previous->lastPeak<.2){for(const Event* e:{previous,&event}){
                size_t sample=std::min(count-1,static_cast<size_t>(std::ceil(e->begin/step)));
                worst(result.horizontalReversal[axis-1],e->peak,.5*radiusAt(axis,e->sign>0,sample),previous->lastPeak,event.firstPeak-previous->lastPeak);}}
            previous=&event;}
    }
    constexpr size_t pairs[3][2]={{0,1},{0,2},{1,2}};
    constexpr Point pairedLimit[]={{.2,1}};
    for(size_t pair=0;pair<3;++pair){std::vector<double> q(count);
        for(size_t i=0;i<count;++i){if(stopped(cancel,i)){result.cancelled=true;return result;}for(size_t axis:pairs[pair]){
            double value=filtered[axis][i]/radiusAt(axis,filtered[axis][i]>=0,i);q[i]+=value*value;}}
        if(!excursions(q,true,step,pairedLimit,result.paired[pair],cancel)){result.cancelled=true;return result;}}
    for(size_t i=0;i<6;++i)reportCase(result.report,result.directional[i],"FORCE_DURATION","Historical upright extended-restraint signed force-duration excursion exceeded");
    reportCase(result.report,result.reducedPositive,"FORCE_POST_UPLIFT","Reduced positive Gz exceeded within 6 s after transition from at least 3 s negative exposure");
    reportCase(result.report,result.zeroToTwo,"FORCE_ZERO_TO_TWO","After sustained nonpositive Gz, transition from 0 to 2 g is shorter than 133 ms");
    if(result.enhancedLongitudinalOnset.utilization>=1)result.report.fail("FORCE_ENHANCED_LONGITUDINAL_ONSET","Padded upper-torso higher negative-Gx exception requires buildup onset below 15 g/s",result.enhancedLongitudinalOnset.startSeconds,result.enhancedLongitudinalOnset.actual,15);
    for(const auto& value:result.horizontalReversal)reportCase(result.report,value,"FORCE_HORIZONTAL_REVERSAL","Consecutive sustained horizontal events reverse between peaks within 200 ms and exceed half the directional 200 ms limit");
    for(const auto& value:result.paired)reportCase(result.report,value,"FORCE_PAIRED","Signed quadrant-ellipse limit exceeded for at least 200 ms");
    for(const auto& value:result.durationExtent)reportCase(result.report,value,"FORCE_DURATION_EXTENT","Sustained force exceeds the published duration extent of the historical profile");
    result.performed=true;return result;
}
}
