//
//  telenav_protobuf_renderer.cpp
//  Mapnik
//
//  Created by Jin, YiQing on 10/25/13.
//  Copyright (c) 2013 Telenav. All rights reserved.
//

// mapnik

#include <mapnik/graphics.hpp>

#include <mapnik/rule.hpp>
#include <mapnik/debug.hpp>
#include <mapnik/layer.hpp>
#include <mapnik/label_collision_detector.hpp>
#include <mapnik/feature_type_style.hpp>
#include <mapnik/marker.hpp>
#include <mapnik/marker_cache.hpp>
#include <mapnik/unicode.hpp>
#include <mapnik/font_set.hpp>
#include <mapnik/parse_path.hpp>
#include <mapnik/map.hpp>
#include <mapnik/svg/svg_converter.hpp>
#include <mapnik/svg/svg_renderer_agg.hpp>
#include <mapnik/svg/svg_path_adapter.hpp>
#include <mapnik/pixel_position.hpp>

#include <mapnik/image_compositing.hpp>
#include <mapnik/image_filter.hpp>
#include <mapnik/image_util.hpp>

#include <mapnik/symbolizer_helpers.hpp>
#include <mapnik/json/geojson_generator.hpp>
#include <mapnik/geom_util.hpp>

// boost
#include <boost/math/special_functions/round.hpp>
#include <boost/algorithm/string/predicate.hpp>

// stl
#include <cmath>

#include <mapnik/well_known_srs.hpp>

#include "mapnik/telenav_protobuf_renderer.h"

using namespace com::telenav::proto;
using namespace com::telenav::proto::map;

namespace mapnik {
// constructor
template<typename T>
tn_renderer<T>::tn_renderer(Map const& m, T & protobuf, double scale_factor,
		unsigned offset_x, unsigned offset_y) :
		feature_style_processor<tn_renderer>(m, scale_factor), protobuf_(
				protobuf), t_(m.width(), m.height(), m.get_current_extent(),
				offset_x, offset_y), style_level_compositing_(false), font_engine_(), font_manager_(
				font_engine_), scale_factor_(scale_factor), detector_(
				std::make_shared<label_collision_detector4>(
						box2d<double>(-m.buffer_size(), -m.buffer_size(),
								m.width() + m.buffer_size(),
								m.height() + m.buffer_size()))), query_extent_() {
	setup(m);
}
template<typename T>
tn_renderer<T>::~tn_renderer() {
}

template<typename T>
void tn_renderer<T>::setup(Map const &m) {

}

template<typename T>
void tn_renderer<T>::start_map_processing(Map const& map) {
	MAPNIK_LOG_DEBUG(tn_renderer)<< "tn_renderer: Start map processing bbox=" << map.get_current_extent();
//        ras_ptr->clip_box(0,0,width_,height_);
}

template<typename T>
void tn_renderer<T>::end_map_processing(Map const&) {

//        agg::rendering_buffer buf(pixmap_.raw_data(),width_,height_, width_ * 4);
//        agg::pixfmt_rgba32_pre pixf(buf);
//        pixf.demultiply();
	MAPNIK_LOG_DEBUG(tn_renderer)<< "tn_renderer: End map processing";
}

template<typename T>
void tn_renderer<T>::start_layer_processing(layer const& lay,
		box2d<double> const& query_extent) {
	MAPNIK_LOG_DEBUG(tn_renderer)<< "tn_renderer: Start processing layer=" << lay.name();
	MAPNIK_LOG_DEBUG(tn_renderer) << "tn_renderer: -- datasource=" << lay.datasource().get();
	MAPNIK_LOG_DEBUG(tn_renderer) << "tn_renderer: -- query_extent=" << query_extent;

	if (lay.clear_label_cache())
	{
		detector_->clear();
	}

	query_extent_ = query_extent;
	boost::optional<box2d<double> > const& maximum_extent = lay.maximum_extent();
	if (maximum_extent)
	{
		query_extent_.clip(*maximum_extent);
	}
}

template<typename T>
void tn_renderer<T>::end_layer_processing(layer const&) {
	MAPNIK_LOG_DEBUG(tn_renderer)<< "tn_renderer: End layer processing";
}

template<typename T>
void tn_renderer<T>::start_style_processing(feature_type_style const& st) {
	MAPNIK_LOG_DEBUG(tn_renderer)<< "tn_renderer: Start processing style";
	if (st.comp_op() || st.image_filters().size() > 0 || st.get_opacity() < 1)
	{
		style_level_compositing_ = true;
	}
	else
	{
		style_level_compositing_ = false;
	}

	if (style_level_compositing_)
	{
		int radius = 0;
		mapnik::filter::filter_radius_visitor visitor(radius);
		for (mapnik::filter::filter_type const& filter_tag : st.image_filters())
		{
			boost::apply_visitor(visitor, filter_tag);
		}
		if (radius > t_.offset())
		{
			t_.set_offset(radius);
		}
	}
	else
	{
		t_.set_offset(0);
	}
}

template<typename T>
void tn_renderer<T>::end_style_processing(feature_type_style const& st) {
	MAPNIK_LOG_DEBUG(tn_renderer)<< "tn_renderer: End processing style";
}

using std::numeric_limits;

template<typename T>
void tn_renderer<T>::process(text_symbolizer const& sym,
		mapnik::feature_impl & feature, proj_transform const& prj_trans) {
	box2d<double> clip_box = clipping_extent();
	text_symbolizer_helper<face_manager<freetype_engine>,
			label_collision_detector4> helper(sym, feature, prj_trans, width_,
			height_, scale_factor_, t_, font_manager_, *detector_, clip_box);

	VectorMapTile & tile = protobuf_;

//	std::string json;
//	mapnik::json::feature_generator g;
//	if (!g.generate(json, feature)) {
//		throw std::runtime_error("Failed to generate GeoJSON");
//	}

	//std::cout.precision(numeric_limits<double>::digits10 + 1);
	//std::cout<< "Feature: " << json << std::endl;

	//std::cout << feature.get("name");

	std::string highway = feature.get("highway").to_string();
	std::string name = feature.get("name").to_string();

	while (helper.next()) {
		placements_type const& placements = helper.placements();
		for (unsigned int ii = 0; ii < placements.size(); ++ii) {
			PointFeature *pf = tile.add_pf();
			pf->set_maintype(PT_ROAD); //for now treat all of them as road label.
			pf->set_subtype("a");
			pf->set_name(name);
			Polyline *spline = pf->mutable_spline();

			text_path const& path = placements[ii];
			//std::cout << "Path: " << ii << std::endl;
			//std::cout << "  Center: " << path.center.x <<","<<path.center.y << std::endl;
			int olat = 0, olon = 0;
			double cx = path.center.x;
			double cy = path.center.y;
			for (std::size_t i = 0; i < path.num_nodes(); ++i) {
				char_info_ptr c;
				double x, y, angle; //pixel position
				path.vertex(c, x, y, angle);
				//char ch = c->c;
				//std::cout << "vertex: " << ch <<","<< x << "," << y << "," << angle << std::endl;
				x += cx;
				y = cy - y;
				t_.backward(&x, &y);
				merc2lonlat(&x, &y, 1);
				//std::cout << "  latlon: " << ch <<","<< x << "," << y <<std::endl;
				int lat = y * 1000000;
				int lon = x * 1000000;
				spline->add_latlon(lat - olat);
				spline->add_latlon(lon - olon);
				olat = lat;
				olon = lon;

			}
		}
	}
}

template<typename T>
void tn_renderer<T>::process(point_symbolizer const& sym,
		mapnik::feature_impl & feature, proj_transform const& prj_trans) {

	std::string kind = feature.get("kind").to_string();
	std::string name = feature.get("name").to_string();
	//std::cout << "Feature(PF): " << kind << "," << feature.get("name") << std::endl;
	VectorMapTile & tile = protobuf_;

	PointFeature *pf = tile.add_pf();
	pf->set_maintype(PT_ROAD);
	pf->set_subtype("a");
	if(!name.empty()){
		pf->set_name(name);
	}else if(!kind.empty()){
		pf->set_name(kind);
	}else{
		pf->set_name("what??");
	}

	for (std::size_t i = 0; i < feature.num_geometries(); ++i) {
		geometry_type const& geom = feature.get_geometry(i);
		double x;
		double y;
		double z = 0;
		if (sym.get_point_placement() == CENTROID_POINT_PLACEMENT) {
			if (!label::centroid(geom, x, y))
				return;
		} else {
			if (!label::interior_position(geom, x, y))
				return;
		}
		prj_trans.backward(x, y, z);
		merc2lonlat(&x, &y, 1);
		int lat = y * 1000000;
		int lon = x * 1000000;
		Polyline *spline = pf->mutable_spline();
		spline->add_latlon(lat);
		spline->add_latlon(lon-300);
		spline->add_latlon(0);
		spline->add_latlon(300);
		spline->add_latlon(0);
		spline->add_latlon(300);
	}
}

template<typename T>
void tn_renderer<T>::process(line_symbolizer const& sym,
		mapnik::feature_impl & feature, proj_transform const& prj_trans) {

	if (!feature.has_key("highway")) {
		return; // only handles highways for now.
	}
	std::string highway = feature.get("highway").to_string();
	std::string name = feature.get("name").to_string();

	//std::cout << "Feature(LF): " << highway << "," << name << std::endl;
//	std::string json;
//	mapnik::json::feature_generator g;
//	if (!g.generate(json, feature)) {
//		throw std::runtime_error("Failed to generate GeoJSON");
//	}
//	std::cout << "Feature: " << json << std::endl;
	VectorMapTile & tile = protobuf_;

	RoadFeature *rf = tile.add_rf();
	rf->set_roadtype(match_tn_road_type(highway));
	rf->set_roadsubtype(RST_COMMON);
	if(!name.empty()){
		rf->set_roadname(name);
	}

	for (geometry_type & geom : feature.paths()) {
		if (geom.size() > 1) {
			Polyline *line = rf->add_lines();
			int olat = 0, olon = 0;
			for (std::size_t i = 0; i < geom.size(); i++) {
				double x, y;
				geom.vertex(&x, &y);
				merc2lonlat(&x, &y, 1);
				//std::cout << "  latlon: " << x << "," << y << std::endl;
				int lat = y * 1000000;
				int lon = x * 1000000;
				line->add_latlon(lat - olat);
				line->add_latlon(lon - olon);
				olat = lat;
				olon = lon;
			}
		}
	}
}
template<typename T>
void tn_renderer<T>::process(line_pattern_symbolizer const& sym,
		mapnik::feature_impl & feature, proj_transform const& prj_trans) {
}
template<typename T>
void tn_renderer<T>::process(polygon_symbolizer const& sym,
		mapnik::feature_impl & feature, proj_transform const& prj_trans) {
	std::string kind = feature.get("kind").to_string();
	//std::cout << "Feature(AF): " << kind << "," << feature.get("osm_id").to_string() << std::endl;
//	std::string json;
//	mapnik::json::feature_generator g;
//	if (!g.generate(json, feature)) {
//		throw std::runtime_error("Failed to generate GeoJSON");
//	}
//  std::cout << "Feature: " << json << std::endl;
	VectorMapTile & tile = protobuf_;
	AreaFeature *af = tile.add_af();
	af->set_maintype(match_tn_area_type(kind));
	af->set_subtype("900150");

	for (geometry_type & geom : feature.paths()) {
		if (geom.size() > 1) {
			Polygon *ring = af->add_rings();
			int olat = 0, olon = 0;
			for (std::size_t i = 0; i < geom.size(); i++) {
				double x, y;
				geom.vertex(&x, &y);
				merc2lonlat(&x, &y, 1);
				//std::cout << "  latlon: " << x << "," << y << std::endl;
				int lat = y * 1000000;
				int lon = x * 1000000;
				ring->add_latlon(lat - olat);
				ring->add_latlon(lon - olon);
				olat = lat;
				olon = lon;
			}
		}
	}
}

template<typename T>
void tn_renderer<T>::process(polygon_pattern_symbolizer const& sym,
		mapnik::feature_impl & feature, proj_transform const& prj_trans) {
}
template<typename T>
void tn_renderer<T>::process(raster_symbolizer const& sym,
		mapnik::feature_impl & feature, proj_transform const& prj_trans) {
}
template<typename T>
void tn_renderer<T>::process(shield_symbolizer const& sym,
		mapnik::feature_impl & feature, proj_transform const& prj_trans) {
}
template<typename T>
void tn_renderer<T>::process(building_symbolizer const& sym,
		mapnik::feature_impl & feature, proj_transform const& prj_trans) {
}
template<typename T>
void tn_renderer<T>::process(markers_symbolizer const& sym,
		mapnik::feature_impl & feature, proj_transform const& prj_trans) {
}
template<typename T>
void tn_renderer<T>::process(debug_symbolizer const& sym,
		feature_impl & feature, proj_transform const& prj_trans) {
}

template<typename T>
void tn_renderer<T>::painted(bool painted) {
	//pixmap_.painted(painted);
}

template<typename T>
RoadType tn_renderer<T>::match_tn_road_type(std::string const& highway) {
	if (highway == "motorway" || highway == "primary") {
		return RT_HIGHWAY;
	} else if (highway == "secondary" || highway == "trunk") {
		return RT_ARTERIAL;
	} else if (highway == "residential" || highway == "tertiary"
			|| highway == "unclassified") {
		return RT_LOCAL;
	} else if (highway == "pedestrian" || highway == "footway") {
		return RT_PEDESTRIAN;
	} else if (highway == "service") {
		return RT_TERMINAL;
	} else if (highway == "residential") {
		return RT_LOCAL;
	}
	return RT_UNKNOWN;
}

template<typename T>
AreaType tn_renderer<T>::match_tn_area_type(std::string const& kind) {
	if (kind == "park" || kind == "forest") {
		return BK_AREA_PARK;
	}else if (kind == "ocean" || kind == "water" || kind == "riverbank" || kind == "lake" || kind == "playa"){
		return BK_AREA_WATER;
	}
	return BK_AREA_UNKNOWN;
}

template<typename T>
std::string tn_renderer<T>::match_tn_point_type(std::string const& highway) const {
	// not implemented yet, for now I just use PT_ROAD for all point features.
	if (highway == "trunk" || highway == "primary"){
		return "h";
	}else if(highway == "secondary" || highway == "tertiary"){
		return "a";
	}else {
		return "s";
	}
}

template class tn_renderer<VectorMapTile> ;
}

