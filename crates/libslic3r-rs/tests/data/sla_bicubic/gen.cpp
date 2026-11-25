// Native vectors for SLA/bicubic.h (R954). The pin's header does not compile as it stands (nothing
// at the pin includes it): bicubic.h:96 `typedef typename KERNEL KernelInternal;` is not valid C++,
// and the lambdas of cubic_interpolate / bicubic_interpolate (bicubic.h:152, 176) neither. This
// generator therefore carries bicubic.h:12-129 verbatim (the three kernels, BicubicInternal::clamp
// and CubicKernel's kernel / interpolate) with only line 96 left out, and compiles them with the
// engine's flags (clang -O3, -ffp-contract=on), so the kernels' arithmetic is the pin's.
// cubic_interpolate / bicubic_interpolate cannot be compiled at the pin and are not compared.
//
// usage: gen > vectors.txt
// Lines: `C <kernel> <float|double> <a00 .. a33 as %a>`, `K <kernel> <type> <x %a> <kernel(x) %a>`,
// `I <kernel> <type> <f0> <f1> <f2> <f3> <x> <interpolate %a>`, `M <a> <lo> <hi> <clamp>` (ints).
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

namespace Slic3r {
// ---- bicubic.h:12-129, verbatim but for line 96 ----
namespace BicubicInternal {
	// Linear kernel, to be able to test cubic methods with hat kernels.
	template<typename T>
	struct LinearKernel
	{
		typedef T	FloatType;

		static T a00() { return T(0.); }
		static T a01() { return T(0.); }
		static T a02() { return T(0.); }
		static T a03() { return T(0.); }
		static T a10() { return T(1.); }
		static T a11() { return T(-1.); }
		static T a12() { return T(0.); }
		static T a13() { return T(0.); }
		static T a20() { return T(0.); }
		static T a21() { return T(1.); }
		static T a22() { return T(0.); }
		static T a23() { return T(0.); }
		static T a30() { return T(0.); }
		static T a31() { return T(0.); }
		static T a32() { return T(0.); }
		static T a33() { return T(0.); }
	};

	// Interpolation kernel aka Catmul-Rom aka Keyes kernel.
	template<typename T>
	struct CubicCatmulRomKernel
	{
		typedef T	FloatType;

		static T a00() { return     0;     }
		static T a01() { return (T)-0.5;   }
		static T a02() { return (T) 1.;    }
		static T a03() { return (T)-0.5;   }
		static T a10() { return (T) 1.;    }
		static T a11() { return     0;     }
		static T a12() { return (T)-5./2.; }
		static T a13() { return (T) 3./2.; }
		static T a20() { return     0;     }
		static T a21() { return (T) 0.5;   }
		static T a22() { return (T) 2.;    }
		static T a23() { return (T)-3./2.; }
		static T a30() { return     0;     }
		static T a31() { return     0;     }
		static T a32() { return (T)-0.5;   }
		static T a33() { return (T) 0.5;   }
	};

	// B-spline kernel
	template<typename T>
	struct CubicBSplineKernel
	{
		typedef T	FloatType;

		static T a00() { return (T)  1./6.; }
		static T a01() { return (T) -3./6.; }
		static T a02() { return (T)  3./6.; }
		static T a03() { return (T) -1./6.; }
		static T a10() { return (T)  4./6.; }
		static T a11() { return      0;     }
		static T a12() { return (T) -6./6.; }
		static T a13() { return (T)  3./6.; }
		static T a20() { return (T)  1./6.; }
		static T a21() { return (T)  3./6.; }
		static T a22() { return (T)  3./6.; }
		static T a23() { return (T)- 3./6.; }
		static T a30() { return      0;     }
		static T a31() { return      0;     }
		static T a32() { return      0;     }
		static T a33() { return (T)  1./6.; }
	};

	template<class T>
	inline T clamp(T a, T lower, T upper)
	{
		return (a < lower) ? lower : 
	   		   (a > upper) ? upper : a;
	}
}

template<typename KERNEL>
struct CubicKernel
{
	typedef typename KERNEL::FloatType		FloatType;

	static FloatType kernel(FloatType x)
	{
		x = fabs(x);
		if (x >= (FloatType)2.)
			return 0.0f;
		if (x <= (FloatType)1.) {
			FloatType x2 = x * x;
			FloatType x3 = x2 * x;
			return KERNEL::a10() + KERNEL::a11() * x + KERNEL::a12() * x2 + KERNEL::a13() * x3;
		}
		assert(x > (FloatType)1. && x < (FloatType)2.);
		x -= (FloatType)1.;
		FloatType x2 = x * x;
		FloatType x3 = x2 * x;
		return KERNEL::a00() + KERNEL::a01() * x + KERNEL::a02() * x2 + KERNEL::a03() * x3;
	}

	static FloatType interpolate(FloatType f0, FloatType f1, FloatType f2, FloatType f3, FloatType x)
	{
		const FloatType  x2 = x*x;
		const FloatType  x3 = x*x*x;
		return f0*(KERNEL::a00() + KERNEL::a01() * x + KERNEL::a02() * x2 + KERNEL::a03() * x3) +
			   f1*(KERNEL::a10() + KERNEL::a11() * x + KERNEL::a12() * x2 + KERNEL::a13() * x3) +
			   f2*(KERNEL::a20() + KERNEL::a21() * x + KERNEL::a22() * x2 + KERNEL::a23() * x3) + 
			   f3*(KERNEL::a30() + KERNEL::a31() * x + KERNEL::a32() * x2 + KERNEL::a33() * x3);
	}
};

// Linear splines
typedef CubicKernel<BicubicInternal::LinearKernel<float>>					LinearKernelf;
typedef CubicKernel<BicubicInternal::LinearKernel<double>>					LinearKerneld;
// ---- end of the copy ----
} // namespace Slic3r

using namespace Slic3r;

static uint64_t g_state = 0x2545F4914F6CDD1Dull;
static uint64_t lcg()
{
    g_state = g_state * 6364136223846793005ull + 1442695040888963407ull;
    return g_state;
}
static double unit() { return double(lcg() >> 11) * (1.0 / 9007199254740992.0); }

template<class T> static const char *tname();
template<> const char *tname<float>() { return "float"; }
template<> const char *tname<double>() { return "double"; }

template<class KERNEL> static void coeffs(const char *name)
{
    typedef typename KERNEL::FloatType T;
    std::printf("C %s %s", name, tname<T>());
    const T a[16] = {KERNEL::a00(), KERNEL::a01(), KERNEL::a02(), KERNEL::a03(), KERNEL::a10(), KERNEL::a11(),
                     KERNEL::a12(), KERNEL::a13(), KERNEL::a20(), KERNEL::a21(), KERNEL::a22(), KERNEL::a23(),
                     KERNEL::a30(), KERNEL::a31(), KERNEL::a32(), KERNEL::a33()};
    for (T v : a) std::printf(" %a", double(v));
    std::printf("\n");
}

template<class KERNEL> static void run(const char *name)
{
    typedef typename KERNEL::FloatType T;
    coeffs<KERNEL>(name);
    for (int i = 0; i < 300; ++i) {
        T x;
        switch (i % 4) {
        case 0: x = T(unit() * 5. - 2.5); break;
        case 1: x = T(unit() * 2. - 1.); break;
        case 2: x = T(int(lcg() % 9) - 4) * T(0.5); break;
        default: x = T(unit() * 1.e-3); break;
        }
        T v = CubicKernel<KERNEL>::kernel(x);
        std::printf("K %s %s %a %a\n", name, tname<T>(), double(x), double(v));
    }
    for (int i = 0; i < 300; ++i) {
        T f[4];
        for (T &v : f) v = T(unit() * 200. - 100.);
        T x = (i % 5 == 0) ? T(int(lcg() % 3)) * T(0.5) : T(unit());
        T v = CubicKernel<KERNEL>::interpolate(f[0], f[1], f[2], f[3], x);
        std::printf("I %s %s %a %a %a %a %a %a\n", name, tname<T>(), double(f[0]), double(f[1]), double(f[2]), double(f[3]), double(x), double(v));
    }
}

int main()
{
    run<BicubicInternal::LinearKernel<float>>("Linear");
    run<BicubicInternal::LinearKernel<double>>("Linear");
    run<BicubicInternal::CubicCatmulRomKernel<float>>("CatmulRom");
    run<BicubicInternal::CubicCatmulRomKernel<double>>("CatmulRom");
    run<BicubicInternal::CubicBSplineKernel<float>>("BSpline");
    run<BicubicInternal::CubicBSplineKernel<double>>("BSpline");
    for (int i = 0; i < 40; ++i) {
        int a = int(lcg() % 21) - 10, lo = int(lcg() % 7) - 3, hi = lo + int(lcg() % 8);
        std::printf("M %d %d %d %d\n", a, lo, hi, BicubicInternal::clamp(a, lo, hi));
    }
    return 0;
}
