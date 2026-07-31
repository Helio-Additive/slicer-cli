// arrange_harness.cpp — proof harness for headless arrange
#include "libslic3r/Arrange.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "nlohmann/json.hpp"
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
using json=nlohmann::json;
using namespace Slic3r;
using namespace Slic3r::arrangement;

static bool load_json_config(const std::string& fp, DynamicPrintConfig& cfg) {
    std::ifstream f(fp); if(!f.is_open())return false;
    try{json j=json::parse(f);ConfigSubstitutionContext ctx(ForwardCompatibilitySubstitutionRule::Enable);
    for(auto&[k,v]:j.items()){
        if(k=="type"||k=="name"||k=="inherits"||k=="from"||k=="setting_id"||k=="instantiation"||k=="description"||k=="compatible_printers"||k=="compatible_prints"||k=="include"||k=="upward_compatible_machine"||k=="printer_model"||k=="printer_variant"||k=="default_filament_profile"||k=="default_print_profile")continue;
        try{std::string vs;
        if(v.is_array()){std::vector<std::string>ps;for(auto&e:v){if(e.is_string())ps.push_back(e.get<std::string>());else if(e.is_number())ps.push_back(std::to_string(e.get<double>()));}for(size_t i=0;i<ps.size();i++){if(i>0)vs+=",";vs+=ps[i];}}
        else if(v.is_string())vs=v.get<std::string>();else if(v.is_number_float())vs=std::to_string(v.get<double>());else if(v.is_number_integer())vs=std::to_string(v.get<int>());else if(v.is_boolean())vs=v.get<bool>()?"1":"0";
        if(!vs.empty()&&vs!="nil")cfg.set_deserialize(k,vs,ctx);}catch(...){}
    }return true;}catch(...){return false;}
}

static ExPolygon make_rect(double w,double h){coord_t W=scaled<coord_t>(w),H=scaled<coord_t>(h);Points pts={Point(0,0),Point(W,0),Point(W,H),Point(0,H)};return ExPolygon(Polygon(pts));}

int run_arrange_spike(const char* dir){
    std::string base=dir;while(!base.empty()&&base.back()=='/')base.pop_back();
    DynamicPrintConfig cfg;
    load_json_config(base+"/BBL/machine/fdm_bbl_3dp_001_common.json",cfg);
    if(!load_json_config(base+"/BBL/machine/Bambu Lab A1 0.4 nozzle.json",cfg))return 2;
    if(!load_json_config(base+"/BBL/process/0.20mm Standard @BBL A1.json",cfg))return 2;
    if(!load_json_config(base+"/BBL/filament/Bambu PLA Basic @BBL A1.json",cfg))return 2;

    ArrangePolygons items;
    {ArrangePolygon a;a.poly=make_rect(30,20);a.name="A";a.height=50;a.brim_width=5;items.push_back(a);}
    {ArrangePolygon b;b.poly=make_rect(40,15);b.name="B";b.height=30;b.brim_width=5;items.push_back(b);}
    {ArrangePolygon c;c.poly=make_rect(25,25);c.name="C";c.height=60;c.brim_width=5;items.push_back(c);}

    ArrangeParams params;
#ifdef ENGINE_ORCA
    params.clearance_radius=cfg.has("extruder_clearance_max_radius")?cfg.opt_float("extruder_clearance_max_radius"):1.0f;
#else
    params.cleareance_radius=cfg.has("extruder_clearance_max_radius")?cfg.opt_float("extruder_clearance_max_radius"):1.0f;
#endif
    params.min_obj_distance=scaled<coord_t>(10.0);
    params.allow_rotations=true;
    params.do_final_align=true;
#ifdef ENGINE_ORCA
    update_arrange_params(params,&cfg,items);
    update_selected_items_inflation(items,&cfg,params);
    Points bed_pts=get_shrink_bedpts(&cfg,params);
#else
    update_arrange_params(params,cfg,items);
    update_selected_items_inflation(items,cfg,params);
    Points bed_pts=get_shrink_bedpts(cfg,params);
#endif

    std::cerr<<"BEFORE:";for(auto&ap:items)std::cerr<<" "<<ap.name<<".bed="<<ap.bed_idx;std::cerr<<std::endl;
    arrange(items,{},bed_pts,params);
    std::cerr<<"AFTER :";for(auto&ap:items)std::cerr<<" "<<ap.name<<".bed="<<ap.bed_idx<<" pos=("<<unscaled<double>(ap.translation.x())<<","<<unscaled<double>(ap.translation.y())<<")";std::cerr<<std::endl;

    std::printf("{\"placed\":[");
    for(size_t i=0;i<items.size();i++){auto&ap=items[i];
        std::printf("{\"n\":\"%s\",\"bed\":%d,\"x\":%.2f,\"y\":%.2f}%s",
            ap.name.c_str(),ap.bed_idx,unscaled<double>(ap.translation.x()),unscaled<double>(ap.translation.y()),i<2?",":"");}
    std::printf("]}\n");
    int p=0;for(auto&ap:items)if(ap.bed_idx==0)p++;
    return p==3?0:1;
}
