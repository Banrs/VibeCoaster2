// Included inside support_assemblies. Bank rotates the short saddle, not the
// entire foundation frame. All mounts share the closed, continuous girder.
inline std::vector<Support> twisted(const Design& d,const SupportRegion& region,double spread,double reach,double depthScale,Cancel cancel){
    return sweptGirder(d,region.begin,region.end,true,spread,reach,depthScale,cancel);
}
