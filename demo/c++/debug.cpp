/*****************************************************************************
 *
 * This file is part of Mapnik (c++ mapping toolkit)
 *
 * Copyright (C) 2011 Artem Pavlenko
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *****************************************************************************/

#include <mapnik/map.hpp>
#include <mapnik/layer.hpp>
#include <mapnik/rule.hpp>
#include <mapnik/line_symbolizer.hpp>
#include <mapnik/polygon_symbolizer.hpp>
#include <mapnik/text_symbolizer.hpp>
#include <mapnik/feature_type_style.hpp>
#include <mapnik/graphics.hpp>
#include <mapnik/datasource_cache.hpp>
#include <mapnik/font_engine_freetype.hpp>
#include <mapnik/agg_renderer.hpp>
#include <mapnik/expression.hpp>
#include <mapnik/color_factory.hpp>
#include <mapnik/image_util.hpp>
#include <mapnik/load_map.hpp>
#include <iostream>
#include <fstream>
#include "mapnik/telenav_protobuf_renderer.h"


int main ( int argc , char** argv)
{
    using namespace mapnik;

    try {
        std::cout << " running demo ... \n";
        std::string mapnik_dir(argv[1]);
        std::cout << " looking for 'postgis' plugin in... " << mapnik_dir << "/lib/mapnik/input/" << "\n";
                datasource_cache::instance().register_datasources(mapnik_dir + "/lib/mapnik/input/");

        std::cout << " looking for DejaVuSans font in... " << mapnik_dir << "/lib/mapnik/fonts/DejaVuSans.ttf" << "\n";
        freetype_engine::register_font(mapnik_dir + "/lib/mapnik/fonts/DejaVuSans.ttf");
        freetype_engine::register_font(mapnik_dir + "/lib/mapnik/fonts/DejaVuSans-Bold.ttf");
        freetype_engine::register_font(mapnik_dir + "/lib/mapnik/fonts/DejaVuSans-BoldOblique.ttf");
        freetype_engine::register_font(mapnik_dir + "/lib/mapnik/fonts/unifont-5.1.20080907.ttf");
        Map m(256,256);
        std::cout << "loading map..." << std:: endl;
        load_map(m,"../../style-sheet/osm-simple.xml");

        //5278, 12709, 15
        m.zoom_to_box(box2d<double>(-13582554.1782,4493274.27072,-13581331.1857,4494497.26317));
        //m.zoom_to_box(box2d<double>(-13589892.1329,4490828.28581,-13580108.1933,4500612.22543));
        VectorMapTile tile;
        tn_renderer<VectorMapTile> ren(m, tile);
        std::cout << "Rendering..." << std:: endl;
        ren.apply();
        std::cout << tile.DebugString() << std::endl;
        std::string msg("These maps have been rendered to protobuf in the current directory:\n");
        msg += "tile.proto";
        std::fstream output("tile.proto", std::ios::out | std::ios::trunc | std::ios::binary);
        if (!tile.SerializeToOstream(&output)) {
            std::cerr << "Failed to write tile." << std::endl;
            return -1;
        }
        std::cout << msg;

    }
    catch ( const std::exception & ex )
    {
        std::cerr << "### std::exception: " << ex.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch ( ... )
    {
        std::cerr << "### Unknown exception." << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
