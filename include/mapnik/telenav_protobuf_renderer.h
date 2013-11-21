//
//  telenav_protobuf_renderer.h
//  Mapnik
//
//  Created by Jin, YiQing on 10/25/13.
//  Copyright (c) 2013 Telenav. All rights reserved.
//

#ifndef __Mapnik__telenav_protobuf_renderer__
#define __Mapnik__telenav_protobuf_renderer__

// mapnik
#include <mapnik/config.hpp>            // for MAPNIK_DECL
#include <mapnik/feature_style_processor.hpp>
#include <mapnik/font_engine_freetype.hpp>  // for face_manager, etc
#include <mapnik/noncopyable.hpp>       // for noncopyable
#include <mapnik/rule.hpp>              // for rule, symbolizers
#include <mapnik/box2d.hpp>     // for box2d
#include <mapnik/color.hpp>     // for color
#include <mapnik/ctrans.hpp>    // for CoordTransform
#include <mapnik/image_compositing.hpp>  // for composite_mode_e
#include <mapnik/pixel_position.hpp>
#include <mapnik/request.hpp>

#include "mapnik/map.vector.pb.h"

// boost

#include <memory>

// fwd declaration to avoid depedence on agg headers
namespace agg { struct trans_affine; }

// fwd declarations to speed up compile
namespace mapnik {
    class Map;
    class feature_impl;
    class feature_type_style;
    class label_collision_detector4;
    class layer;
    class marker;
    class proj_transform;
    struct rasterizer;
}

using namespace com::telenav::proto::map;

namespace mapnik {
    template <typename T>
    class MAPNIK_DECL tn_renderer : public feature_style_processor<tn_renderer<T> >,
    private mapnik::noncopyable
    {
        
    public:
        typedef T buffer_type;
        typedef tn_renderer<T> processor_impl_type;
        // create with default, empty placement detector
        tn_renderer(Map const& m, T & protobuf,double scale_factor=1.0, unsigned offset_x=0, unsigned offset_y=0);
        ~tn_renderer();
        
        void start_map_processing(Map const& map);
        void end_map_processing(Map const& map);
        void start_layer_processing(layer const& lay, box2d<double> const& query_extent);
        void end_layer_processing(layer const& lay);
        
        void start_style_processing(feature_type_style const& st);
        void end_style_processing(feature_type_style const& st);
        
        void process(point_symbolizer const& sym,
                     mapnik::feature_impl & feature,
                     proj_transform const& prj_trans);
        void process(line_symbolizer const& sym,
                     mapnik::feature_impl & feature,
                     proj_transform const& prj_trans);
        void process(line_pattern_symbolizer const& sym,
                     mapnik::feature_impl & feature,
                     proj_transform const& prj_trans);
        void process(polygon_symbolizer const& sym,
                     mapnik::feature_impl & feature,
                     proj_transform const& prj_trans);
        void process(polygon_pattern_symbolizer const& sym,
                     mapnik::feature_impl & feature,
                     proj_transform const& prj_trans);
        void process(raster_symbolizer const& sym,
                     mapnik::feature_impl & feature,
                     proj_transform const& prj_trans);
        void process(shield_symbolizer const& sym,
                     mapnik::feature_impl & feature,
                     proj_transform const& prj_trans);
        void process(text_symbolizer const& sym,
                     mapnik::feature_impl & feature,
                     proj_transform const& prj_trans);
        void process(building_symbolizer const& sym,
                     mapnik::feature_impl & feature,
                     proj_transform const& prj_trans);
        void process(markers_symbolizer const& sym,
                     mapnik::feature_impl & feature,
                     proj_transform const& prj_trans);
        void process(debug_symbolizer const& sym,
                     feature_impl & feature,
                     proj_transform const& prj_trans);
        
        inline bool process(rule::symbolizers const& /*syms*/,
                            mapnik::feature_impl & /*feature*/,
                            proj_transform const& /*prj_trans*/)
        {
            // agg renderer doesn't support processing of multiple symbolizers.
            return false;
        }
        
        void painted(bool painted);
        inline eAttributeCollectionPolicy attribute_collection_policy() const
        {
            return DEFAULT;
        }
        
        inline double scale_factor() const
        {
            return scale_factor_;
        }
        
        inline box2d<double> clipping_extent() const
        {
            if (t_.offset() > 0)
            {
                box2d<double> box = query_extent_;
                double scale = static_cast<double>(query_extent_.width())/static_cast<double>(width_);
                // 3 is used here because at least 3 was needed for the 'style-level-compositing-tiled-0,1' visual test to pass
                // TODO - add more tests to hone in on a more robust #
                scale *= t_.offset()*3;
                box.pad(scale);
                return box;
            }
            return query_extent_;
        }
        
    protected:
        
    private:
        buffer_type & protobuf_;
        std::shared_ptr<buffer_type> internal_buffer_;
        mutable buffer_type * current_buffer_;
        CoordTransform t_;
        mutable bool style_level_compositing_;
        unsigned width_;
        unsigned height_;
        freetype_engine font_engine_;
        face_manager<freetype_engine> font_manager_;
        double scale_factor_;
        std::shared_ptr<label_collision_detector4> detector_;
        box2d<double> query_extent_;
        void setup(Map const& m);
        RoadType  match_tn_road_type(std::string const& highway);
        AreaType  match_tn_area_type(std::string const& highway);
        std::string match_tn_point_type(std::string const& highway) const;
    };
}

#endif /* defined(__Mapnik__telenav_protobuf_renderer__) */


