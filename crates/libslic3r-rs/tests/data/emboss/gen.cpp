// Reference vectors for crates/libslic3r-rs/src/emboss.rs, taken from Emboss.cpp at the pin.
//
// The program compiles Emboss.cpp into itself (so that the functions of its anonymous namespace
// can be called, and with REMOVE_SMALL_ISLANDS defined so that remove_small_islands is compiled;
// the define changes nothing else in the file) and links the engine's libslic3r for the rest.
// The three remove_spikes of `#ifdef REMOVE_SPIKES` cannot be compiled at the pin (they are
// declared inside the anonymous namespace and defined outside it, so the calls are ambiguous). Build
// it as a libslic3r test program of a configured Bambu Studio engine build tree, then:
//   ./emboss_gen crates/libslic3r-rs/tests/data/emboss/NotoSansThai-Regular.ttf \
//       | gzip -9n > crates/libslic3r-rs/tests/data/emboss/vectors.txt.gz
//
// One case per line, `TAG inputs -> outputs`. Integers are decimal, doubles and floats the hex of
// their bits. A polygon is `n x y ...`, expolygons are `k` then per expolygon the contour, the
// number of holes and the holes. Texts are the hex of their UTF-8 bytes (`-` when empty).
// The font is Noto Sans Thai Regular (SIL Open Font License 1.1, NotoSansThai-OFL.txt beside it).

#define REMOVE_SMALL_ISLANDS
#include "libslic3r/Emboss.cpp"

#include <cstdio>
#include <cstring>

using namespace Slic3r;
using namespace Slic3r::Emboss;

static uint64_t s = 0x94D049BB133111EBull;
static uint64_t rnd() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return s; }
static double   rndd(double lo, double hi) { return lo + (hi - lo) * double(rnd() % 1000001) / 1000000.; }
static unsigned long long db(double d) { unsigned long long u; memcpy(&u, &d, 8); return u; }
static unsigned fb(float f) { unsigned u; memcpy(&u, &f, 4); return u; }

static void out(double d) { printf(" %016llx", db(d)); }
static void outf(float f) { printf(" %08x", fb(f)); }
static void out(const Vec3d &v) { out(v.x()); out(v.y()); out(v.z()); }
static void out(const Polygon &p)
{
    printf(" %zu", p.points.size());
    for (const Point &pt : p.points) printf(" %lld %lld", (long long) pt.x(), (long long) pt.y());
}
static void out(const Polygons &ps)
{
    printf(" %zu", ps.size());
    for (const Polygon &p : ps) out(p);
}
static void out(const ExPolygons &es)
{
    printf(" %zu", es.size());
    for (const ExPolygon &e : es) { out(e.contour); out(e.holes); }
}
static void out(const Transform3d &t)
{
    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) out(t.matrix()(r, c));
}
static void out_text(const std::string &t)
{
    if (t.empty()) { printf(" -"); return; }
    printf(" ");
    for (unsigned char c : t) printf("%02x", c);
}

// Random polygons on a grid coarse enough that points coincide and edges cross.
static Polygons random_polygons(int c)
{
    int64_t grid  = (c % 4 == 0) ? 10 : (c % 4 == 1) ? 1000 : (c % 4 == 2) ? 100000 : 7;
    int     cells = 6 + int(rnd() % 10);
    Polygons polys(1 + rnd() % 3);
    for (Polygon &p : polys) {
        size_t n = 3 + rnd() % 8;
        for (size_t i = 0; i < n; ++i)
            p.points.emplace_back(coord_t(int64_t(rnd() % cells) * grid), coord_t(int64_t(rnd() % cells) * grid));
    }
    return polys;
}

// Rectangles and stars: valid rings that touch, overlap and nest.
static Polygons shape_polygons(int c)
{
    int64_t  grid = (c % 3 == 0) ? 100 : (c % 3 == 1) ? 2500 : 100000;
    Polygons polys(1 + rnd() % 4);
    for (Polygon &p : polys) {
        int64_t x = int64_t(rnd() % 12) * grid, y = int64_t(rnd() % 12) * grid;
        int64_t w = int64_t(1 + rnd() % 8) * grid, h = int64_t(1 + rnd() % 8) * grid;
        if (rnd() % 3 == 0) {
            // a star: long thin tips
            int    tips = 3 + int(rnd() % 5);
            double r1 = double(w), r2 = double(w) / (2. + double(rnd() % 20));
            for (int i = 0; i < 2 * tips; ++i) {
                double a = M_PI * i / tips, r = (i % 2 == 0) ? r1 : r2;
                p.points.emplace_back(coord_t(x + int64_t(r * cos(a))), coord_t(y + int64_t(r * sin(a))));
            }
        } else {
            p.points = {Point(x, y), Point(x + w, y), Point(x + w, y + h), Point(x, y + h)};
            if (rnd() % 4 == 0) p.reverse();
        }
    }
    return polys;
}

static void out_healed(const HealedExPolygons &h) { printf(" %d", int(h.is_healed)); out(h.expolygons); }

static void out_prop(const FontProp &p)
{
    if (p.char_gap) printf(" %d", *p.char_gap); else printf(" -");
    if (p.line_gap) printf(" %d", *p.line_gap); else printf(" -");
    if (p.boldness) outf(*p.boldness); else printf(" -");
    if (p.skew) outf(*p.skew); else printf(" -");
    printf(" %d %d", int(p.align.first), int(p.align.second));
    outf(p.size_in_mm);
}

static FontProp make_prop(int c)
{
    FontProp p(float(1 + rnd() % 30) / (1 + rnd() % 3));
    if (c % 3 == 1) p.char_gap = int(rnd() % 400) - 100;
    if (c % 4 == 2) p.line_gap = int(rnd() % 600) - 200;
    if (c % 5 == 3) p.boldness = float(rndd(-0.4, 0.8));
    if (c % 7 == 4) p.skew = float(rndd(-0.5, 0.5));
    p.align = FontProp::Align(FontProp::HorizontalAlign(rnd() % 3), FontProp::VerticalAlign(rnd() % 3));
    return p;
}

static std::string narrow(const std::wstring &w) { return boost::nowide::narrow(w); }

// Eigen::AngleAxis::toRotationMatrix calls sin and cos of one angle, which the compiler of the
// reference build may merge into one call (__sincos_stret on macOS) whose sine is, for a few
// angles, one ulp away from sin(). Which of the two the engine runs is the compiler's choice, not
// Emboss.cpp's: an angle where they differ is not given to the test.
static double (*volatile sin_fn)(double) = std::sin;
static double (*volatile cos_fn)(double) = std::cos;
static bool sincos_same(double angle)
{
    Matrix3d m = Eigen::AngleAxisd(angle, Vec3d::UnitZ()).toRotationMatrix();
    return m(1, 0) == sin_fn(angle) && m(0, 0) == cos_fn(angle);
}
// The two angles create_transformation_onto_surface turns by (Emboss.cpp:1946-1969).
static bool onto_surface_sincos_same(const Vec3d &normal, double up_limit)
{
    Vec3d up_dir = Vec3d::UnitY(), emboss_dir = Vec3d::UnitZ();
    Vec3d wanted_up_dir = suggest_up(normal, up_limit);
    Vec3d axis_view;
    double angle_view;
    if (normal == -Vec3d::UnitZ()) {
        axis_view = Vec3d::UnitY();
        angle_view = M_PI;
    } else {
        axis_view = emboss_dir.cross(normal);
        angle_view = std::acos(emboss_dir.dot(normal));
        axis_view.normalize();
    }
    Eigen::AngleAxis view_rot(angle_view, axis_view);
    Vec3d wanterd_up_rotated = view_rot.matrix().inverse() * wanted_up_dir;
    wanterd_up_rotated.normalize();
    double angle_up = std::acos(up_dir.dot(wanterd_up_rotated));
    return sincos_same(angle_view) && sincos_same(angle_up) && sincos_same(-angle_up);
}

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: emboss_gen <NotoSansThai-Regular.ttf>\n"); return 1; }
    const char *font_path = argv[1];

    // ---- heal_polygons, heal_expolygons, union_with_delta, divide_segments_for_close_point
    for (int c = 0; c < 300; ++c) {
        Polygons polys     = (c % 5 < 3) ? random_polygons(c) : shape_polygons(c);
        bool     non_zero  = (c / 2) % 2 == 0;
        unsigned max_iter  = (c % 9 == 8) ? 1 + unsigned(rnd() % 3) : 10;
        float    delta     = (c % 3 == 0) ? 50.f : float(1 + rnd() % 3000);
        double   distance  = (c % 2 == 0) ? 0.6 : double(1 + rnd() % 20) / 4.;
        printf("HEAL %d %u", int(non_zero), max_iter);
        outf(delta); out(distance); out(polys);
        printf(" ->");
        HealedExPolygons healed = heal_polygons(polys, non_zero, max_iter);
        out_healed(healed);
        // every polygon as an expolygon of its own: not a valid ExPolygons, which is what heal is for
        ExPolygons raw;
        for (const Polygon &p : polys) raw.emplace_back(p);
        {
            ExPolygons e = raw;
            bool ok = heal_expolygons(e, max_iter);
            printf(" %d", int(ok)); out(e);
        }
        out_healed(Emboss::union_with_delta(healed.expolygons, delta, max_iter));
        {
            ExPolygons e = healed.expolygons;
            bool ok = divide_segments_for_close_point(e, distance);
            printf(" %d", int(ok)); out(e);
        }
        {
            ExPolygons e = raw;
            bool ok = divide_segments_for_close_point(e, distance);
            printf(" %d", int(ok)); out(e);
        }
        printf("\n");
    }

    // ---- the helpers of the anonymous namespace
    for (int c = 0; c < 250; ++c) {
        Polygons polys = (c % 2 == 0) ? random_polygons(c) : shape_polygons(c);
        ExPolygons raw;
        for (const Polygon &p : polys) raw.emplace_back(p);
        double bevel = (c % 3 == 0) ? 2.56 : rndd(2.5, 400.);
        double pixel = (c % 3 == 0) ? 6. : rndd(0.5, 12.);
        double area  = rndd(0., 50.) * rndd(0., 1e9);
        unsigned max_iter = 1 + unsigned(rnd() % 10);
        printf("PRIV"); out(bevel); out(pixel); out(area); printf(" %u", max_iter); out(polys);
        printf(" ->");
        SpikeDesc sd(bevel, pixel);
        // remove_when_spike on every index of the first polygon, each on a fresh copy
        printf(" %zu", polys.front().size());
        for (size_t i = 0; i < polys.front().size(); ++i) {
            Polygon p = polys.front();
            bool r = remove_when_spike(p, i, sd);
            printf(" %d", int(r)); out(p);
        }
        { ExPolygons e = raw; Points d = collect_duplicates(to_points(e)); remove_spikes_in_duplicates(e, d); out(e); }
        { ExPolygons e = raw; ExPolygon r = create_bounding_rect(e); out(r.contour); out(r.holes); }
        { ExPolygons e = raw; remove_small_islands(e, area); out(e); }
        { ExPolygons e = raw; bool ok = heal_dupl_inter(e, max_iter); printf(" %d", int(ok)); out(e); }
        printf("\n");
    }

    // ---- get_align_x_offset
    for (int c = 0; c < 200; ++c) {
        BoundingBox a, b;
        if (c % 7 != 6) { a.merge(Point(int(rnd() % 20000) - 10000, int(rnd() % 20000) - 10000)); a.merge(Point(int(rnd() % 20000) - 10000, int(rnd() % 20000) - 10000)); }
        if (c % 5 != 4) { b.merge(Point(int(rnd() % 20000) - 10000, int(rnd() % 20000) - 10000)); b.merge(Point(int(rnd() % 20000) - 10000, int(rnd() % 20000) - 10000)); }
        int h = c % 3;
        printf("ALIGNX %d %d %lld %lld %lld %lld %d %lld %lld %lld %lld -> %d\n", h, int(a.defined), (long long) a.min.x(), (long long) a.min.y(), (long long) a.max.x(), (long long) a.max.y(),
               int(b.defined), (long long) b.min.x(), (long long) b.min.y(), (long long) b.max.x(), (long long) b.max.y(),
               get_align_x_offset(FontProp::HorizontalAlign(h), a, b));
    }

    // ---- line counts and ranges
    for (int c = 0; c < 200; ++c) {
        std::wstring w;
        size_t n = rnd() % 12;
        static const wchar_t alphabet[] = {L'a', L'\n', L' ', L'\t', L'\r', L'b', 0x0E01, L'\n', 0x1F600};
        for (size_t i = 0; i < n; ++i) w.push_back(alphabet[rnd() % 9]);
        ExPolygonsWithIds shapes;
        for (wchar_t ch : w) shapes.push_back({unsigned(ch), {}});
        std::string t = narrow(w);
        printf("LINES"); out_text(t);
        printf(" -> %u %u %u", get_count_lines(w), get_count_lines(t), get_count_lines(shapes));
        LineRanges r = get_line_ranges(shapes);
        printf(" %zu", r.size());
        for (auto &p : r) printf(" %zu %zu", p.first, p.second);
        std::wstring nd = remove_duplicates(w);
        out_text(narrow(nd));
        printf("\n");
    }

    // ---- the font
    {
        std::unique_ptr<FontFile> ff = create_font_file(font_path);
        if (!ff) { fprintf(stderr, "cannot load %s\n", font_path); return 1; }
        printf("FONT -> %zu", ff->infos.size());
        for (const FontFile::Info &i : ff->infos) printf(" %d %d %d %d", i.ascent, i.descent, i.linegap, i.unit_per_em);
        printf(" %d %d\n", int(is_italic(*ff, 0)), int(is_italic(*ff, 1)));

        // every code point of some ranges, at two flatnesses
        std::vector<int> cps;
        for (int cp = 0x20; cp < 0x180; ++cp) cps.push_back(cp);
        for (int cp = 0x0E00; cp < 0x0E80; ++cp) cps.push_back(cp);
        for (int cp = 0x2000; cp < 0x2070; ++cp) cps.push_back(cp);
        for (int cp : {0x25CC, 0xFEFF, 0xFFFD, 0x1F600, 0x10FFFF}) cps.push_back(cp);
        for (int cp : cps)
            for (float flatness : {1.5f, 20.f}) {
                printf("GLYPH %d", cp); outf(flatness); printf(" ->");
                std::optional<Glyph> g = letter2glyph(*ff, 0, cp, flatness);
                if (!g) { printf(" none\n"); break; }
                printf(" %d %d", g->advance_width, g->left_side_bearing); out(g->shape);
                printf("\n");
            }
        printf("GLYPH1 %d", 0x0E01); outf(1.f); printf(" -> %s\n", letter2glyph(*ff, 1, 0x0E01, 1.f) ? "some" : "none");

        // texts
        std::vector<std::wstring> texts = {
            L"", L"\n", L" ", L"a", L"ก", L"กขฃ", L"สวัสดี", L"ก ข\nคฅฆ",
            L"ก\tข\r\nจุด\n\nง", L"12 3๐๑", L"ก ข​ฃ ค", L"เกียรติ\nก้าวไกล นำ",
            L"\nก\n", L"xกy中ข", L"กกกกกกกกกกกก", L"​  \t",
        };
        for (const std::wstring &w : texts) {
            std::string t = narrow(w);
            bool unknown = false;
            std::string r = create_range_text(t, *ff, 0, &unknown);
            printf("RANGE"); out_text(t); printf(" ->"); out_text(r); printf(" %d", int(unknown));
            {
                std::unique_ptr<FontFile> f2 = create_font_file(font_path);
                std::vector<std::shared_ptr<const FontFile>> fonts;
                fonts.emplace_back(std::move(f2));
                fonts.emplace_back(create_font_file(font_path));
                std::string t2 = t; bool u2 = false;
                std::string r2 = create_range_text(t2, fonts, 0, &u2);
                out_text(r2); printf(" %d", int(u2));
            }
            printf(" %s\n", create_range_text(t, *ff, 1, nullptr).empty() ? "e" : "n");
        }

        for (int c = 0; c < 80; ++c) {
            const std::wstring &w = texts[c % texts.size()];
            std::string  t   = narrow(w);
            FontProp     prop = make_prop(c);
            bool         bfc  = (c / 3) % 2 == 1;
            bool         cancel = c % 37 == 36;
            double       scale = get_text_shape_scale(prop, *ff);
            unsigned     lines = 1 + unsigned(rnd() % 4);
            int          tx = int(rnd() % 1000), ty = -int(rnd() % 1000);
            printf("TEXT"); out_prop(prop); printf(" %d %d %u %d %d", int(bfc), int(cancel), lines, tx, ty); out_text(t);
            printf(" ->");
            out(scale);
            printf(" %d", get_line_height(*ff, prop));
            outf(get_align_y_offset(prop.align.second, lines, *ff, prop));
            out(get_align_y_offset_in_mm(prop.align.second, lines, *ff, prop));

            FontFileWithCache font(create_font_file(font_path));
            BackFontCacheFn fn = nullptr;
            if (bfc)
                fn = [font_path]() {
                    std::vector<FontFileWithCache> r;
                    r.emplace_back();
                    r.emplace_back(create_font_file(font_path));
                    return r;
                };
            auto was_canceled = [cancel]() { return cancel; };
            EmbossShape es;
            text2vshapes(es, font, w, prop, scale, was_canceled, fn);
            printf(" %zu", es.shapes_with_ids.size());
            for (const ExPolygonsWithId &e : es.shapes_with_ids) { printf(" %u %d", e.id, int(e.is_healed)); out(e.expoly); }
            printf(" %zu", es.text_align_offsets.size());
            for (const Vec2f &v : es.text_align_offsets) { outf(v.x()); outf(v.y()); }
            printf(" %zu", es.text_scales.size());
            for (float v : es.text_scales) outf(v);
            printf(" %zu", es.text_cursors.size());
            for (float v : es.text_cursors) outf(v);
            printf(" %zu", es.text_absolute_cursors.size());
            for (float v : es.text_absolute_cursors) outf(v);
            printf(" %d %d", es.align_type.first, es.align_type.second);
            outf(es.line_height); outf(es.first_line_offset_y);
            printf(" %u", get_count_lines(es.shapes_with_ids));
            {
                BoundingBox bb = get_extents(es.shapes_with_ids);
                printf(" %d", int(bb.defined));
                if (bb.defined) printf(" %lld %lld %lld %lld", (long long) bb.min.x(), (long long) bb.min.y(), (long long) bb.max.x(), (long long) bb.max.y());
                ExPolygonsWithIds copy = es.shapes_with_ids;
                if (bb.defined) {
                    center(copy);
                    translate(copy, Point(tx, ty));
                }
                for (const ExPolygonsWithId &e : copy) out(e.expoly);
            }
            // Slic3r::union_with_delta(EmbossShape&), twice: the second call returns the stored shape
            {
                EmbossShape es2 = es;
                ExPolygons u = Slic3r::union_with_delta(es2, UNION_DELTA, UNION_MAX_ITERATIN);
                out(u); printf(" %d", int(es2.final_shape.is_healed));
                ExPolygons u2 = Slic3r::union_with_delta(es2, 1.f, 1);
                out(u2);
            }
            // text2shapes, on a font of its own
            {
                FontFileWithCache font2(create_font_file(font_path));
                EmbossShape es3;
                HealedExPolygons h = text2shapes(es3, font2, t.c_str(), prop, scale, was_canceled, fn);
                out_healed(h);
                printf(" %zu", es3.shapes_with_ids.size());
            }
            printf("\n");
        }
    }

    // ---- projections
    for (int c = 0; c < 150; ++c) {
        Point  p(int(rnd() % 2000000) - 1000000, int(rnd() % 2000000) - 1000000);
        Vec3d  v(rndd(-50, 50), rndd(-50, 50), rndd(-50, 50));
        double depth = rndd(-5, 20), scale = rndd(0.0001, 3.);
        Transform3d tr = Transform3d::Identity();
        tr.translate(Vec3d(rndd(-10, 10), rndd(-10, 10), rndd(-10, 10)));
        tr.rotate(Eigen::AngleAxisd(rndd(-3, 3), Vec3d(rndd(-1, 1), rndd(-1, 1), rndd(0.1, 1)).normalized()));
        tr.scale(Vec3d(rndd(0.2, 3), rndd(0.2, 3), rndd(0.2, 3)));
        Vec3d dir(rndd(-2, 2), rndd(-2, 2), (c % 9 == 8) ? 0. : rndd(0.5, 5));
        printf("PROJ %lld %lld", (long long) p.x(), (long long) p.y()); out(v); out(depth); out(scale); out(tr); out(dir);
        printf(" ->");
        auto dump = [&](const IProjection &pr) {
            auto fb2 = pr.create_front_back(p);
            out(fb2.first); out(fb2.second);
            out(pr.project(v));
            double d = 1.25;
            std::optional<Vec2d> u = pr.unproject(v, &d);
            if (u) { printf(" 1"); out(u->x()); out(u->y()); } else printf(" 0");
            out(d);
            std::optional<Vec2d> u2 = pr.unproject(v);
            if (u2) { printf(" 1"); out(u2->x()); out(u2->y()); } else printf(" 0");
        };
        dump(ProjectZ(depth));
        dump(ProjectScale(std::make_unique<ProjectZ>(depth), scale));
        dump(ProjectTransform(std::make_unique<ProjectZ>(depth), tr));
        dump(ProjectTransform(std::make_unique<ProjectScale>(std::make_unique<ProjectZ>(depth), scale), tr));
        dump(OrthoProject(tr, dir));
        out(OrthoProject3d(dir).project(v));
        printf("\n");
    }

    // ---- suggest_up, calc_up, create_transformation_onto_surface, apply_transformation
    for (int c = 0; c < 260; ++c) {
        Vec3d normal(rndd(-1, 1), rndd(-1, 1), rndd(-1, 1));
        if (c % 6 == 0) normal = Vec3d(0, 0, (c % 12 == 0) ? 1 : -1);
        else if (c % 6 == 1) normal = Vec3d(rndd(-0.05, 0.05), rndd(-0.05, 0.05), (c % 12 == 1) ? 1 : -1);
        normal.normalize();
        double limit = (c % 3 == 0) ? 0.9 : rndd(0.1, 0.99);
        Vec3d  position(rndd(-100, 100), rndd(-100, 100), rndd(-100, 100));
        Transform3d tr = Transform3d::Identity();
        tr.translate(Vec3d(rndd(-10, 10), rndd(-10, 10), rndd(-10, 10)));
        if (c % 8 != 7) tr.rotate(Eigen::AngleAxisd(rndd(-3, 3), Vec3d(rndd(-1, 1), rndd(-1, 1), rndd(-1, 1)).normalized()));
        if (c % 3 == 2) tr.scale(Vec3d(rndd(0.2, 3), rndd(0.2, 3), rndd(0.2, 3)));
        float angle = float(rndd(-4, 4)), distance = float(rndd(-10, 10));
        int   which = c % 4;
        if (!onto_surface_sincos_same(normal, limit) || !sincos_same(double(angle)))
            continue;
        printf("UP"); out(normal); out(limit); out(position); out(tr); outf(angle); outf(distance); printf(" %d", which);
        printf(" ->");
        out(suggest_up(normal, limit));
        std::optional<float> up = calc_up(tr, limit);
        if (up) { printf(" 1"); outf(*up); } else printf(" 0");
        out(create_transformation_onto_surface(position, normal, limit));
        Transform3d t2 = tr;
        apply_transformation((which & 1) ? std::optional<float>(angle) : std::nullopt, (which & 2) ? std::optional<float>(distance) : std::nullopt, t2);
        out(t2);
        printf("\n");
    }

    // ---- calculate_angle(s), sample_slice
    for (int c = 0; c < 400; ++c) {
        Polygon poly;
        {
            // a ring around the origin
            size_t n = 3 + rnd() % 14;
            for (size_t i = 0; i < n; ++i) {
                double a = 2 * M_PI * i / n, r = rndd(2e5, 9e5);
                poly.points.emplace_back(coord_t(r * cos(a)), coord_t(r * sin(a)));
            }
            if (c % 5 == 4) poly.reverse();
        }
        size_t n_pp = 1 + rnd() % 4;
        PolygonPoints pps;
        for (size_t i = 0; i < n_pp; ++i) {
            size_t idx = rnd() % poly.size();
            const Point &a = poly[idx], &b = poly[(idx + 1) % poly.size()];
            double t = (rnd() % 4 == 0) ? 0. : rndd(0, 1);
            pps.push_back({idx, Point(coord_t(a.x() + t * (b.x() - a.x())), coord_t(a.y() + t * (b.y() - a.y())))});
        }
        int32_t distance = int32_t(1 + rnd() % ((c % 3 == 0) ? 3000000 : 300000));
        // bounding boxes of letters along x, some undefined
        BoundingBoxes bbs;
        {
            size_t n = rnd() % 9;
            // (widths that, times the scale, are distances along the ring)
            int    x = -int(rnd() % 4000);
            for (size_t i = 0; i < n; ++i) {
                BoundingBox bb;
                int w = 1 + int(rnd() % 1500);
                if (rnd() % 5 != 0) { bb.merge(Point(x, -int(rnd() % 1000))); bb.merge(Point(x + w, int(rnd() % 90000))); }
                bbs.push_back(bb);
                x += w + int(rnd() % 200);
            }
        }
        double scale = rndd(0.0005, 0.02);
        printf("ANGLE %d", distance); out(scale); out(poly);
        printf(" %zu", pps.size());
        for (const PolygonPoint &pp : pps) printf(" %zu %lld %lld", pp.index, (long long) pp.point.x(), (long long) pp.point.y());
        printf(" %zu", bbs.size());
        for (const BoundingBox &bb : bbs) {
            printf(" %d", int(bb.defined));
            if (bb.defined) printf(" %lld %lld %lld %lld", (long long) bb.min.x(), (long long) bb.min.y(), (long long) bb.max.x(), (long long) bb.max.y());
        }
        printf(" ->");
        out(calculate_angle(distance, pps.front(), poly));
        for (double a : calculate_angles(distance, pps, poly)) out(a);
        TextLine line{poly, pps.front(), 0.f};
        PolygonPoints samples = sample_slice(line, bbs, scale);
        printf(" %zu", samples.size());
        for (const PolygonPoint &pp : samples) printf(" %zu %lld %lld", pp.index, (long long) pp.point.x(), (long long) pp.point.y());
        printf("\n");
    }

    // ---- polygons2model
    for (int c = 0; c < 80; ++c) {
        Polygons polys = shape_polygons(c);
        ExPolygons shape = heal_polygons(polys).expolygons;
        if (c % 4 == 3 && !shape.empty()) {
            // two squares that share one corner, the first at the corner of the bounding box:
            // points that coincide (the duplicit branch), no edge twice
            BoundingBox bb = get_extents(shape);
            coord_t g = 1000 + coord_t(rnd() % 5000);
            Point m = bb.max;
            for (int k = 0; k < 2; ++k) {
                Polygon sq;
                sq.points = {m, Point(m.x() + g, m.y()), Point(m.x() + g, m.y() + g), Point(m.x(), m.y() + g)};
                shape.emplace_back(sq);
                m += Point(g, g);
            }
        }
        double depth = rndd(0.1, 5.), scale = rndd(0.0001, 0.01);
        printf("MODEL"); out(depth); out(scale); out(shape);
        printf(" ->");
        ProjectScale pr(std::make_unique<ProjectZ>(depth), scale);
        indexed_triangle_set its = polygons2model(shape, pr);
        printf(" %zu", its.vertices.size());
        for (const Vec3f &v : its.vertices) { outf(v.x()); outf(v.y()); outf(v.z()); }
        printf(" %zu", its.indices.size());
        for (const Vec3i32 &t : its.indices) printf(" %d %d %d", t.x(), t.y(), t.z());
        printf("\n");
    }
    return 0;
}
