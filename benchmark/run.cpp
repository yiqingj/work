#include <mapnik/graphics.hpp>
#include <mapnik/image_data.hpp>
#include <mapnik/image_util.hpp>
#include <mapnik/image_reader.hpp>
#include <mapnik/util/conversions.hpp>


// stl
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <set>
#include <stdexcept>
#include <thread>
#include <vector>
#include <memory>

// boost
#include <boost/version.hpp>
#include <boost/bind.hpp>

#define BOOST_CHRONO_HEADER_ONLY
#include <boost/chrono/process_cpu_clocks.hpp>
#include <boost/chrono.hpp>

using namespace boost::chrono;
using namespace mapnik;

static unsigned test_num = 1;
static bool dry_run = false;
static std::set<int> test_set;

typedef process_cpu_clock clock_type;
typedef clock_type::duration dur;

template <typename T>
void benchmark(T & test_runner, std::string const& name)
{
    try {
        bool should_run_test = true;
        if (!test_set.empty())
        {
            should_run_test = test_set.find(test_num) != test_set.end();
        }
        if (should_run_test || dry_run)
        {
            if (!test_runner.validate())
            {
                std::clog << "test did not validate: " << name << "\n";
                //throw std::runtime_error(std::string("test did not validate: ") + name);
            }
            if (dry_run)
            {
                std::clog << test_num << ") " << (test_runner.threads_ ? "threaded -> ": "")
                    << name << "\n";
            }
            else
            {
                process_cpu_clock::time_point start;
                dur elapsed;
                if (test_runner.threads_ > 0)
                {
                    typedef std::vector<std::unique_ptr<std::thread> > thread_group;
                    typedef thread_group::value_type value_type;
                    thread_group tg;
                    for (unsigned i=0;i<test_runner.threads_;++i)
                    {
                        tg.emplace_back(new std::thread(test_runner));
                    }
                    start = process_cpu_clock::now();
                    std::for_each(tg.begin(), tg.end(), [](value_type & t) {if (t->joinable()) t->join();});
                    elapsed = process_cpu_clock::now() - start;
                }
                else
                {
                    start = process_cpu_clock::now();
                    test_runner();
                    elapsed = process_cpu_clock::now() - start;
                }
                std::clog << test_num << ") " << (test_runner.threads_ ? "threaded -> ": "")
                    << name << ": "
                    << boost::chrono::duration_cast<milliseconds>(elapsed) << "\n";
            }
        }
    }
    catch (std::exception const& ex)
    {
        std::clog << "test runner did not complete: " << ex.what() << "\n";
    }
    test_num++;
}

bool compare_images(std::string const& src_fn,std::string const& dest_fn)
{
    std::unique_ptr<mapnik::image_reader> reader1(mapnik::get_image_reader(dest_fn,"png"));
    if (!reader1.get())
    {
        throw mapnik::image_reader_exception("Failed to load: " + dest_fn);
    }
    std::shared_ptr<image_32> image_ptr1 = std::make_shared<image_32>(reader1->width(),reader1->height());
    reader1->read(0,0,image_ptr1->data());

    std::unique_ptr<mapnik::image_reader> reader2(mapnik::get_image_reader(src_fn,"png"));
    if (!reader2.get())
    {
        throw mapnik::image_reader_exception("Failed to load: " + src_fn);
    }
    std::shared_ptr<image_32> image_ptr2 = std::make_shared<image_32>(reader2->width(),reader2->height());
    reader2->read(0,0,image_ptr2->data());

    image_data_32 const& dest = image_ptr1->data();
    image_data_32 const& src = image_ptr2->data();

    unsigned int width = src.width();
    unsigned int height = src.height();
    if ((width != dest.width()) || height != dest.height()) return false;
    for (unsigned int y = 0; y < height; ++y)
    {
        const unsigned int* row_from = src.getRow(y);
        const unsigned int* row_to = dest.getRow(y);
        for (unsigned int x = 0; x < width; ++x)
        {
           if (row_from[x] != row_to[x]) return false;
        }
    }
    return true;
}

struct test1
{
    unsigned iter_;
    unsigned threads_;
    explicit test1(unsigned iterations, unsigned threads=0) :
      iter_(iterations),
      threads_(threads)
      {}

    bool validate()
    {
        return true;
    }

    void operator()()
    {
        mapnik::image_data_32 im(256,256);
        std::string out;
        for (unsigned i=0;i<iter_;++i) {
            out.clear();
            out = mapnik::save_to_string(im,"png");
        }
    }
};

struct test2
{
    unsigned iter_;
    unsigned threads_;
    std::shared_ptr<image_32> im_;
    explicit test2(unsigned iterations, unsigned threads=0) :
      iter_(iterations),
      threads_(threads),
      im_()
    {
        std::string filename("./benchmark/data/multicolor.png");
        std::unique_ptr<mapnik::image_reader> reader(mapnik::get_image_reader(filename,"png"));
        if (!reader.get())
        {
            throw mapnik::image_reader_exception("Failed to load: " + filename);
        }
        im_ = std::make_shared<image_32>(reader->width(),reader->height());
        reader->read(0,0,im_->data());
    }

    bool validate()
    {
        std::string expected("./benchmark/data/multicolor-hextree-expected.png");
        std::string actual("./benchmark/data/multicolor-hextree-actual.png");
        mapnik::save_to_file(im_->data(),actual, "png8:m=h");
        return compare_images(actual,expected);
    }

    void operator()()
    {
        std::string out;
        for (unsigned i=0;i<iter_;++i) {
            out.clear();
            out = mapnik::save_to_string(im_->data(),"png8:m=h");
        }
    }
};


struct test3
{
    unsigned iter_;
    unsigned threads_;
    double val_;
    explicit test3(unsigned iterations, unsigned threads=0) :
      iter_(iterations),
      threads_(threads),
      val_(-0.123) {}
    bool validate()
    {
        std::ostringstream s;
        s << val_;
        return (s.str() == "-0.123");
    }
    void operator()()
    {
        std::string out;
        for (unsigned i=0;i<iter_;++i) {
            std::ostringstream s;
            s << val_;
            out = s.str();
        }
    }
};

struct test4
{
    unsigned iter_;
    unsigned threads_;
    double val_;
    explicit test4(unsigned iterations, unsigned threads=0) :
      iter_(iterations),
      threads_(threads),
      val_(-0.123) {}

    bool validate()
    {
        std::string s;
        mapnik::util::to_string(s,val_);
        return (s == "-0.123");
    }
    void operator()()
    {
        std::string out;
        for (unsigned i=0;i<iter_;++i) {
            out.clear();
            mapnik::util::to_string(out,val_);
        }
    }
};


struct test5
{
    unsigned iter_;
    unsigned threads_;
    double val_;
    explicit test5(unsigned iterations, unsigned threads=0) :
      iter_(iterations),
      threads_(threads),
      val_(-0.123) {}

    bool validate()
    {
        std::string s;
        to_string_impl(s,val_);
        return (s == "-0.123");
    }
    bool to_string_impl(std::string &s , double val)
    {
        s.resize(s.capacity());
        while (true)
        {
            size_t n2 = static_cast<size_t>(snprintf(&s[0], s.size()+1, "%g", val));
            if (n2 <= s.size())
            {
                s.resize(n2);
                break;
            }
            s.resize(n2);
        }
        return true;
    }
    void operator()()
    {
        std::string out;
        for (unsigned i=0;i<iter_;++i)
        {
            out.clear();
            to_string_impl(out , val_);
        }
    }
};


#include <mapnik/box2d.hpp>
#include <mapnik/projection.hpp>
#include <mapnik/proj_transform.hpp>

struct test6
{
    unsigned iter_;
    unsigned threads_;
    std::string src_;
    std::string dest_;
    mapnik::box2d<double> from_;
    mapnik::box2d<double> to_;
    bool defer_proj4_init_;
    explicit test6(unsigned iterations,
                   unsigned threads,
                   std::string const& src,
                   std::string const& dest,
                   mapnik::box2d<double> from,
                   mapnik::box2d<double> to,
                   bool defer_proj) :
      iter_(iterations),
      threads_(threads),
      src_(src),
      dest_(dest),
      from_(from),
      to_(to),
      defer_proj4_init_(defer_proj) {}

    bool validate()
    {
        mapnik::projection src(src_,defer_proj4_init_);
        mapnik::projection dest(dest_,defer_proj4_init_);
        mapnik::proj_transform tr(src,dest);
        mapnik::box2d<double> bbox = from_;
        if (!tr.forward(bbox)) return false;
        return ((std::fabs(bbox.minx() - to_.minx()) < .5) &&
                (std::fabs(bbox.maxx() - to_.maxx()) < .5) &&
                (std::fabs(bbox.miny() - to_.miny()) < .5) &&
                (std::fabs(bbox.maxy() - to_.maxy()) < .5)
               );
    }
    void operator()()
    {
        unsigned count=0;
        for (int i=-180;i<180;++i)
        {
            for (int j=-85;j<85;++j)
            {
                mapnik::projection src(src_,defer_proj4_init_);
                mapnik::projection dest(dest_,defer_proj4_init_);
                mapnik::proj_transform tr(src,dest);
                mapnik::box2d<double> box(i,j,i,j);
                if (!tr.forward(box)) throw std::runtime_error("could not transform coords");
                ++count;
            }
        }
    }
};

#include <mapnik/unicode.hpp>
#include <mapnik/expression.hpp>
#include <mapnik/expression_string.hpp>

struct test7
{
    unsigned iter_;
    unsigned threads_;
    std::string expr_;
    explicit test7(unsigned iterations,
                   unsigned threads,
                   std::string const& expr) :
      iter_(iterations),
      threads_(threads),
      expr_(expr)
      {}

    bool validate()
    {
        mapnik::expression_ptr expr = mapnik::parse_expression(expr_,"utf-8");
        return mapnik::to_expression_string(*expr) == expr_;
    }
    void operator()()
    {
         for (unsigned i=0;i<iter_;++i) {
             mapnik::expression_ptr expr = mapnik::parse_expression(expr_,"utf-8");
         }
    }
};

#include <mapnik/expression_grammar.hpp>

struct test8
{
    unsigned iter_;
    unsigned threads_;
    std::string expr_;
    explicit test8(unsigned iterations,
                   unsigned threads,
                   std::string const& expr) :
      iter_(iterations),
      threads_(threads),
      expr_(expr)
      {}

    bool validate()
    {
         transcoder tr("utf-8");
         mapnik::expression_grammar<std::string::const_iterator> expr_grammar(tr);
         mapnik::expression_ptr expr = mapnik::parse_expression(expr_,expr_grammar);
         return mapnik::to_expression_string(*expr) == expr_;
    }
    void operator()()
    {
         transcoder tr("utf-8");
         mapnik::expression_grammar<std::string::const_iterator> expr_grammar(tr);
         for (unsigned i=0;i<iter_;++i) {
             mapnik::expression_ptr expr = mapnik::parse_expression(expr_,expr_grammar);
         }
    }
};

#include "agg_conv_clip_polygon.h"
#include <mapnik/wkt/wkt_factory.hpp>
#include <mapnik/util/geometry_to_wkt.hpp>
#include <mapnik/geometry.hpp>

#include "agg_conv_clip_polygon.h"

struct test11a
{
    unsigned iter_;
    unsigned threads_;
    std::string wkt_in_;
    mapnik::box2d<double> extent_;
    typedef agg::conv_clip_polygon<mapnik::geometry_type> conv_clip;
    test11a(unsigned iterations,
           unsigned threads,
           std::string wkt_in,
           mapnik::box2d<double> const& extent)
        : iter_(iterations),
          threads_(threads),
          wkt_in_(wkt_in),
          extent_(extent) {

    }

    bool validate()
    {
        std::string expected_wkt("Polygon((181 286.666667,233 454,315 340,421 446,463 324,559 466,631 321.320755,631 234.386861,528 178,394 229,329 138,212 134,183 228,200 264,181 238.244444),(313 190,440 256,470 248,510 305,533 237,613 263,553 397,455 262,405 378,343 287,249 334,229 191,313 190,313 190))");
        boost::ptr_vector<geometry_type> paths;
        if (!mapnik::from_wkt(wkt_in_, paths))
        {
            throw std::runtime_error("Failed to parse WKT");
        }
        for (geometry_type & geom : paths)
        {
            conv_clip clipped(geom);
            clipped.clip_box(
                        extent_.minx(),
                        extent_.miny(),
                        extent_.maxx(),
                        extent_.maxy());
            unsigned cmd;
            double x,y;
            mapnik::geometry_type geom2(mapnik::geometry_type::types::Polygon);
            while ((cmd = clipped.vertex(&x, &y)) != SEG_END) {
                geom2.push_vertex(x,y,(mapnik::CommandType)cmd);
            }
            std::string wkt;
            bool result = mapnik::util::to_wkt(wkt,geom2);
            if (result) {
                return (wkt == expected_wkt);
            }
        }
        return false;
    }
    void operator()()
    {
        boost::ptr_vector<geometry_type> paths;
        if (!mapnik::from_wkt(wkt_in_, paths))
        {
            throw std::runtime_error("Failed to parse WKT");
        }
        for (unsigned i=0;i<iter_;++i)
        {
            for (geometry_type & geom : paths)
            {
                conv_clip clipped(geom);
                clipped.clip_box(
                            extent_.minx(),
                            extent_.miny(),
                            extent_.maxx(),
                            extent_.maxy());
                unsigned cmd;
                double x,y;
                while ((cmd = clipped.vertex(&x, &y)) != SEG_END) {}
            }
        }
    }
};

#include "agg_conv_clipper.h"
#include "agg_path_storage.h"

struct test11
{
    unsigned iter_;
    unsigned threads_;
    std::string wkt_in_;
    mapnik::box2d<double> extent_;
    typedef agg::conv_clipper<mapnik::geometry_type, agg::path_storage> poly_clipper;
    test11(unsigned iterations,
           unsigned threads,
           std::string wkt_in,
           mapnik::box2d<double> const& extent)
        : iter_(iterations),
          threads_(threads),
          wkt_in_(wkt_in),
          extent_(extent) {

    }

    bool validate()
    {
        std::string expected_wkt("Polygon((212 134,329 138,394 229,528 178,631 234.4,631 321.3,559 466,463 324,421 446,315 340,233 454,181 286.7,181 238.2,200 264,183 228),(313 190,229 191,249 334,343 287,405 378,455 262,553 397,613 263,533 237,510 305,470 248,440 256))");
        boost::ptr_vector<geometry_type> paths;
        if (!mapnik::from_wkt(wkt_in_, paths))
        {
            throw std::runtime_error("Failed to parse WKT");
        }
        agg::path_storage ps;
        ps.move_to(extent_.minx(), extent_.miny());
        ps.line_to(extent_.minx(), extent_.maxy());
        ps.line_to(extent_.maxx(), extent_.maxy());
        ps.line_to(extent_.maxx(), extent_.miny());
        ps.close_polygon();
        for (geometry_type & geom : paths)
        {
            poly_clipper clipped(geom,ps,
                                 agg::clipper_and,
                                 agg::clipper_non_zero,
                                 agg::clipper_non_zero,
                                 1);
            clipped.rewind(0);
            unsigned cmd;
            double x,y;
            mapnik::geometry_type geom2(mapnik::geometry_type::types::Polygon);
            while ((cmd = clipped.vertex(&x, &y)) != SEG_END) {
                geom2.push_vertex(x,y,(mapnik::CommandType)cmd);
            }
            std::string wkt;
            bool result = mapnik::util::to_wkt(wkt,geom2);
            if (result) {
                return (wkt == expected_wkt);
            }
        }
        return false;
    }
    void operator()()
    {
        boost::ptr_vector<geometry_type> paths;
        if (!mapnik::from_wkt(wkt_in_, paths))
        {
            throw std::runtime_error("Failed to parse WKT");
        }
        agg::path_storage ps;
        ps.move_to(extent_.minx(), extent_.miny());
        ps.line_to(extent_.minx(), extent_.maxy());
        ps.line_to(extent_.maxx(), extent_.maxy());
        ps.line_to(extent_.maxx(), extent_.miny());
        ps.close_polygon();
        for (unsigned i=0;i<iter_;++i)
        {
            for (geometry_type & geom : paths)
            {
                poly_clipper clipped(geom,ps,
                                     agg::clipper_and,
                                     agg::clipper_non_zero,
                                     agg::clipper_non_zero,
                                     1);
                clipped.rewind(0);
                unsigned cmd;
                double x,y;
                while ((cmd = clipped.vertex(&x, &y)) != SEG_END) {}
            }
        }
    }
};

#include <mapnik/polygon_clipper.hpp>

struct test12
{
    unsigned iter_;
    unsigned threads_;
    std::string wkt_in_;
    mapnik::box2d<double> extent_;
    typedef mapnik::polygon_clipper<mapnik::geometry_type> poly_clipper;
    test12(unsigned iterations,
           unsigned threads,
           std::string wkt_in,
           mapnik::box2d<double> const& extent)
        : iter_(iterations),
          threads_(threads),
          wkt_in_(wkt_in),
          extent_(extent)
    {
    }

    bool validate()
    {
        std::string expected_wkt("Polygon((181 286.666667,233 454,315 340,421 446,463 324,559 466,631 321.320755,631 234.386861,528 178,394 229,329 138,212 134,183 228,200 264,181 238.244444,181 286.666667),(313 190,440 256,470 248,510 305,533 237,613 263,553 397,455 262,405 378,343 287,249 334,229 191,313 190))");
        boost::ptr_vector<geometry_type> paths;
        if (!mapnik::from_wkt(wkt_in_, paths))
        {
            throw std::runtime_error("Failed to parse WKT");
        }
        for ( geometry_type & geom : paths)
        {
            poly_clipper clipped(extent_, geom);
            unsigned cmd;
            double x,y;
            mapnik::geometry_type geom2(mapnik::geometry_type::types::Polygon);
            while ((cmd = clipped.vertex(&x, &y)) != SEG_END) {
                geom2.push_vertex(x,y,(mapnik::CommandType)cmd);
            }
            std::string wkt;
            bool result = mapnik::util::to_wkt(wkt,geom2);
            if (result) {
                return (wkt == expected_wkt);
            }
        }
        return false;
    }
    void operator()()
    {
        boost::ptr_vector<geometry_type> paths;
        if (!mapnik::from_wkt(wkt_in_, paths))
        {
            throw std::runtime_error("Failed to parse WKT");
        }
        for (unsigned i=0;i<iter_;++i)
        {
            for ( geometry_type & geom : paths)
            {
                poly_clipper clipped(extent_, geom);
                unsigned cmd;
                double x,y;
                while ((cmd = clipped.vertex(&x, &y)) != SEG_END) {}
            }
        }
    }
};

#include <mapnik/font_engine_freetype.hpp>
#include <boost/format.hpp>
struct test13
{
    unsigned iter_;
    unsigned threads_;

    test13(unsigned iterations,
           unsigned threads)
        : iter_(iterations),
          threads_(threads)
    {}

    bool validate()
    {
        return true;
    }

    void operator()()
    {
        mapnik::freetype_engine engine;
        unsigned long count = 0;
        for (unsigned i=0;i<iter_;++i)
        {
            for ( std::string const& name : mapnik::freetype_engine::face_names())
            {
                mapnik::face_ptr f = engine.create_face(name);
                if (f) ++count;
            }
        }
    }
};

#include <mapnik/map.hpp>
#include <mapnik/load_map.hpp>
#include <mapnik/agg_renderer.hpp>
#include <mapnik/datasource_cache.hpp>

struct test14
{
    unsigned iter_;
    unsigned threads_;
    std::string xml_;
    mapnik::box2d<double> extent_;
    test14(unsigned iterations,
           unsigned threads,
           std::string const& xml,
           mapnik::box2d<double> const& extent)
        : iter_(iterations),
          threads_(threads),
          xml_(xml),
          extent_(extent)
    {}

    bool validate()
    {
        mapnik::Map m(256,256);
        mapnik::load_map(m,xml_);
        m.zoom_to_box(extent_);
        mapnik::image_32 im(m.width(),m.height());
        mapnik::agg_renderer<mapnik::image_32> ren(m,im);
        ren.apply();
        //mapnik::save_to_file(im,"test.png");
        return true;
    }

    void operator()()
    {
        mapnik::Map m(256,256);
        mapnik::load_map(m,xml_);
        m.zoom_to_box(extent_);
        for (unsigned i=0;i<iter_;++i)
        {
            mapnik::image_32 im(m.width(),m.height());
            mapnik::agg_renderer<mapnik::image_32> ren(m,im);
            ren.apply();
        }
    }
};

int main( int argc, char** argv)
{
    if (argc > 0) {
        for (int i=0;i<argc;++i) {
            std::string opt(argv[i]);
            if (opt == "-d" || opt == "--dry-run") {
                dry_run = true;
            } else if (opt[0] != '-') {
                int arg;
                if (mapnik::util::string2int(opt,arg)) {
                    test_set.insert(arg);
                }
            }
        }
    }
    mapnik::datasource_cache::instance().register_datasources("./plugins/input/");
    try
    {
        std::cout << "starting benchmark…\n";

        {
            test1 runner(100);
            benchmark(runner,"encoding blank image as png");
        }

        {
            test2 runner(100);
            benchmark(runner,"encoding multicolor image as png8:m=h");
        }

        {
            test1 runner(10,10);
            benchmark(runner,"encoding blank image as png");
        }

        {
            test2 runner(10,10);
            benchmark(runner,"encoding multicolor image as png8:m=h");
        }

        {
            test3 runner(1000000);
            benchmark(runner,"double to string conversion with std::ostringstream");
        }

        {
            test4 runner(1000000);
            benchmark(runner,"double to string conversion with mapnik::util_to_string");
        }

        {
            test5 runner(1000000);
            benchmark(runner,"double to string conversion with snprintf");
        }

        {
            test3 runner(1000000,10);
            benchmark(runner,"double to string conversion with std::ostringstream");
        }

        {
            test4 runner(1000000,10);
            benchmark(runner,"double to string conversion with mapnik::util_to_string");
        }

        {
            test5 runner(1000000,10);
            benchmark(runner,"double to string conversion with snprintf");
        }

        mapnik::box2d<double> from(-180,-80,180,80);
        mapnik::box2d<double> to(-20037508.3427892476,-15538711.0963092316,20037508.3427892476,15538711.0963092316);

        {
            // echo -180 -60 | cs2cs -f "%.10f" +init=epsg:4326 +to +init=epsg:3857
            test6 runner(100000000,100,
                         "+init=epsg:4326",
                         "+init=epsg:3857",
                         from,to,true);
            benchmark(runner,"lonlat -> merc coord transformation (epsg)");
        }

        {
            test6 runner(100000000,100,
                         "+init=epsg:3857",
                         "+init=epsg:4326",
                         to,from,true);
            benchmark(runner,"merc -> lonlat coord transformation (epsg)");
        }

        {
            test6 runner(100000000,100,
                         "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs",
                         "+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0.0 +k=1.0 +units=m +nadgrids=@null +wktext +no_defs +over",
                         from,to,true);
            benchmark(runner,"lonlat -> merc coord transformation (literal)");
        }

        {
            test6 runner(100000000,100,
                         "+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0.0 +k=1.0 +units=m +nadgrids=@null +wktext +no_defs +over",
                         "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs",
                         to,from,true);
            benchmark(runner,"merc -> lonlat coord transformation (literal)");
        }

        {
            test7 runner(10000,100,"([foo]=1)");
            benchmark(runner,"expression parsing with grammer per parse");
        }

        {
            test8 runner(10000,100,"([foo]=1)");
            benchmark(runner,"expression parsing by re-using grammar");
        }

        // TODO - consider bring back rule cache benchmarks after c++11 is in master
        // previously test 9/10

        // polygon/rect clipping
        // IN : POLYGON ((155 203, 233 454, 315 340, 421 446, 463 324, 559 466, 665 253, 528 178, 394 229, 329 138, 212 134, 183 228, 200 264, 155 203),(313 190, 440 256, 470 248, 510 305, 533 237, 613 263, 553 397, 455 262, 405 378, 343 287, 249 334, 229 191, 313 190))
        // RECT : POLYGON ((181 106, 181 470, 631 470, 631 106, 181 106))
        // OUT (expected)
        // POLYGON ((181 286.6666666666667, 233 454, 315 340, 421 446, 463 324, 559 466, 631 321.3207547169811, 631 234.38686131386862, 528 178, 394 229, 329 138, 212 134, 183 228, 200 264, 181 238.24444444444444, 181 286.6666666666667),(313 190, 440 256, 470 248, 510 305, 533 237, 613 263, 553 397, 455 262, 405 378, 343 287, 249 334, 229 191, 313 190))

        mapnik::box2d<double> clipping_box(181,106,631,470);

        {
            std::string filename_("benchmark/data/polygon.wkt");
            std::ifstream in(filename_.c_str(),std::ios_base::in | std::ios_base::binary);
            if (!in.is_open())
                throw std::runtime_error("could not open: '" + filename_ + "'");
            std::string wkt_in( (std::istreambuf_iterator<char>(in) ),
                       (std::istreambuf_iterator<char>()) );
            test11a runner(10000,10,wkt_in,clipping_box);
            benchmark(runner,"clipping polygon with conv_clip_polygon");
        }

        {
            std::string filename_("benchmark/data/polygon.wkt");
            std::ifstream in(filename_.c_str(),std::ios_base::in | std::ios_base::binary);
            if (!in.is_open())
                throw std::runtime_error("could not open: '" + filename_ + "'");
            std::string wkt_in( (std::istreambuf_iterator<char>(in) ),
                       (std::istreambuf_iterator<char>()) );
            test11 runner(10000,10,wkt_in,clipping_box);
            benchmark(runner,"clipping polygon with agg_conv_clipper");
        }


        {
            std::string filename_("benchmark/data/polygon.wkt");
            std::ifstream in(filename_.c_str(),std::ios_base::in | std::ios_base::binary);
            if (!in.is_open())
                throw std::runtime_error("could not open: '" + filename_ + "'");
            std::string wkt_in( (std::istreambuf_iterator<char>(in) ),
                       (std::istreambuf_iterator<char>()) );
            test12 runner(10000,10,wkt_in,clipping_box);
            benchmark(runner,"clipping polygon with mapnik::polygon_clipper");
        }

        {
            bool success = mapnik::freetype_engine::register_fonts("./fonts", true);
            if (!success) {
               std::clog << "warning, did not register any new fonts!\n";
            }
            unsigned face_count = mapnik::freetype_engine::face_names().size();
            test13 runner(1000,10);
            benchmark(runner, (boost::format("font_engine: created %ld faces in ") % (face_count * 1000 * 10)).str());
        }

        {
            test14 runner(500,10,
                          "benchmark/data/polygon_rendering_clip.xml",
                          mapnik::box2d<double>(-20037508.3428,-8317435.0606,20037508.3428,18399242.7298));
            benchmark(runner, "rendering polygon with clipping at full extent");
        }

        {
            test14 runner(500,10,
                          "benchmark/data/polygon_rendering_no_clip.xml",
                          mapnik::box2d<double>(-20037508.3428,-8317435.0606,20037508.3428,18399242.7298));
            benchmark(runner, "rendering polygon without clipping at full extent");
        }

        {
            // note: bbox below is for 16/10491/22911.png
            test14 runner(500,10,
                          "benchmark/data/polygon_rendering_clip.xml",
                          mapnik::box2d<double>(-13622912.929097254,6026906.8062295765,-13621689.93664469,6028129.79868214));
            benchmark(runner, "rendering polygon with clipping at z16 extent");
        }

        {
            test14 runner(500,10,
                          "benchmark/data/polygon_rendering_no_clip.xml",
                          mapnik::box2d<double>(-13622912.929097254,6026906.8062295765,-13621689.93664469,6028129.79868214));
            benchmark(runner, "rendering polygon without clipping at z16 extent");
        }

        std::cout << "...benchmark done\n";
        return 0;
    }
    catch (std::exception const& ex)
    {
        std::clog << "test error: " << ex.what() << "\n";
        return -1;
    }
}
