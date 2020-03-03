/*------------------------*/
/*        rayt.h          */
/*------------------------*/

#include <memory>
#include <iostream>
#include <vector>

#define NUM_THREAD 8

#include <float.h> // FLT_MIN, FLT_MAX
#define PI 3.14159265359f
#define PI2 6.28318530718f
#define RECIP_PI 0.31830988618f
#define RECIP_PI2 0.15915494f
#define LOG2 1.442695f
#define EPSILON 1e-6f
#define GAMMA_FACTOR 2.2f
#define MAX_DEPTH 50


/*-------------------------- [0,1]の一様乱数を返す -----------------------------*/
#include <random>
inline float drand48() {
	return float(((double)(rand()) / (RAND_MAX))); /* RAND_MAX = 32767 */
}

inline float pow2(float x) { return x * x; }
inline float pow3(float x) { return x * x * x; }
inline float pow4(float x) { return x * x * x * x; }
inline float pow5(float x) { return x * x * x * x * x; }
inline float clamp(float x, float a, float b) { return x < a ? a : x > b ? b : x; }
inline float saturate(float x) { return x < 0.f ? 0.f : x > 1.f ? 1.f : x; }
inline float recip(float x) { return 1.f / x; }
inline float mix(float a, float b, float t) { return a * (1.f - t) + b * t; }
inline float step(float edge, float x) { return (x < edge) ? 0.f : 1.f; }
inline float smoothstep(float a, float b, float t) {if (a >= b) return 0.f;	float x = saturate((t - a) / (b - a)); return x * x * (3.f - 2.f * t);}

/* 度数からラジアンに変換 */
inline float radians(float deg) { return (deg / 180.f) * PI; }

/* ラジアンから度数に変換 */
inline float degrees(float rad) { return (rad / PI) * 180.f; }
inline float nearyeq(float a, float b, float eps = EPSILON) { return fabsf(a - b) <= eps; }
inline float iszero(float a) { return fabsf(a) <= EPSILON; };
inline float safe_recip(float x) { return 1.f / (x + EPSILON); }
//inline float safe_recip(float x) { return 1.0f / (iszero(x)?EPSILON:x); }



/*----------------------------------------------------------------------------*/
#include "vectormath_aos.h"
using namespace Vectormath::Aos;
typedef Vector3 vec3;
typedef Vector3 col3;
/*----------------------------------------------------------------------------*/

/* 画像ファイルに描画するための宣言 */
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image.h"
#include "stb_image_write.h"

/*----------------------------------------------------------------------------*/
/* ピクセルに色(RGB)を描画 */
namespace rayt {

	/*------------------ ガンマ補正 ---------------------*/
	/*---------- リニア空間とsRGB空間との変換式 ---------*/
	inline vec3 linear_to_gamma(const vec3& v, float gammaFactor) {
		float recipGammaFactor = recip(gammaFactor);
		return vec3(
			powf(v.getX(), recipGammaFactor),
			powf(v.getY(), recipGammaFactor),
			powf(v.getZ(), recipGammaFactor));
	}
	inline vec3 gamma_to_linear(const vec3& v, float gammaFactor) {
		return vec3(
			powf(v.getX(), gammaFactor),
			powf(v.getY(), gammaFactor),
			powf(v.getZ(), gammaFactor));
	}
	
	/*------------------- 反射ベクトル ------------------*/
	inline vec3 reflect(const vec3& v, const vec3& n) {
		return v - 2.f * dot(v, n) * n;
	}

	/*------------------- 屈折ベクトル ------------------*/
	inline bool refract(const vec3& v, const vec3& n, float ni_over_nt, vec3& refracted) {
		vec3 uv = normalize(v);
		float dt = dot(uv, n);
		float D = 1.f - pow2(ni_over_nt) * (1.f - pow2(dt));
		// Dは判別式.(η1/η2) < 1 でθ1は[0, 90]
		if (D > 0.f) {
			refracted = -ni_over_nt * (uv - n * dt) - n * sqrt(D);
			return true;
		}
		else { // (η1/η2) > 1 の場合全反射
			return false;
		}
	}
	
	/*-------- フレネルの方程式(Schlickの近似値) --------*/
	inline float schlick(float cosine, float ri) {
		float r0 = pow2((1.f - ri) / (1.f + ri));
		return r0 + (1.f - r0) * pow5(1.f - cosine);
	}

	/*------- 球状マッピングしたテクスチャ座標を取得 -------*/
	inline void get_sphere_uv(const vec3& p, float& u, float& v) {
		float phi = atan2(p.getZ(), p.getX());
		float theta = asin(p.getY());
		u = 1.f - (phi + PI) / (2.f * PI);
		v = (theta + PI / 2.f) / PI;
	}



	/*---------- 一様乱数を使ってベクトルを作成 ---------*/
	inline vec3 random_vector() {
		return vec3(drand48(), drand48(), drand48());
	}
	
	/*------------ 単位球の中の任意の点を生成 -----------*/
	inline vec3 random_in_unit_sphere() {
		vec3 p;
		do {
			p = 2.f * random_vector() - vec3(1.f);
		} while (lengthSqr(p) >= 1.f);
		return p;
	}

	/*--- 表面の法線を中心に半球上の無作為な方向を生成 ---*/
	inline vec3 random_cosin_sirection() {
		float r1 = drand48();
		float r2 = drand48();
		float z = sqrt(1.f - r2);
		float phi = PI2 * r1;
		float x = cos(phi) * sqrt(r2);
		float y = sin(phi) * sqrt(r2);
		return vec3(x, y, z);
	}



	/*------------------フィルター処理-------------------*/
	class ImageFilter {
	public:
		// 色を受け取ってフィルター処理した色を返す
		virtual vec3 filter(const vec3& c) const = 0;
	};

	/*------------------ トーンマッピング ------------------*/
	// HDR から LDR に変換するクランプ処理([0,1]に切り詰める)
	class TonemapFilter : public ImageFilter {
	public:
		TonemapFilter() {}
		virtual vec3 filter(const vec3& c) const override {
			return minPerElem(maxPerElem(c, Vector3(0.f)), Vector3(1.f));;
		}
	};
	
	/*-------- ガンマ補正を受けるフィルタークラス -------*/
	class GammaFilter : public ImageFilter {
	public:
		GammaFilter(float factor) : m_factor(factor) {}
		virtual vec3 filter(const vec3& c) const override {
			return linear_to_gamma(c, m_factor);
		}
	private:
		float m_factor;
	};

	/*---------------------------------------------------*/
	class Image {
	public:
		struct rgb {
			unsigned char r;
			unsigned char g;
			unsigned char b;
			//unsigned char a;
		};

		Image() : m_pixels(nullptr) { }

		// フィルターリスト
		Image(int w, int h) {
			m_width = w;
			m_height = h;
			m_pixels.reset(new rgb[m_width * m_height]);
			m_filters.push_back(std::make_unique<GammaFilter>(GAMMA_FACTOR));
			m_filters.push_back(std::make_unique<TonemapFilter>());
		}

		int width() const { return m_width; }
		int height() const { return m_height; }
		void* pixels() const { return m_pixels.get(); }

		// 色を書く
		void write(int x, int y, float r, float g, float b) {
			vec3 c(r, g, b);
			for (auto& f : m_filters) {
				c = f->filter(c);
			}
			int index = m_width * y + x;
			m_pixels[index].r = static_cast<unsigned char>(c.getX() * 255.99f);
			m_pixels[index].g = static_cast<unsigned char>(c.getY() * 255.99f);
			m_pixels[index].b = static_cast<unsigned char>(c.getZ() * 255.99f);
		}

	private:
		int m_width;
		int m_height;
		std::unique_ptr<rgb[]> m_pixels;
		std::vector< std::unique_ptr<ImageFilter> > m_filters;
	};


	/*-------------------------------------------------------------------------*/
	// 光線 : 始点と向きを持つ
	// o→(origin)：始点ベクトル
	// d→(direction)：方向ベクトル
	/*-------------------------------------------------------------------------*/
	class Ray {
	public:
		Ray() {}
		Ray(const vec3& o, const vec3& dir)
			: m_origin(o)
			, m_direction(dir) {
		}

		const vec3& origin() const { return m_origin; }
		const vec3& direction() const { return m_direction; }

		// 始点 o→ から方向 d→ に向かう直線上の任意の点 p→ を，パラメータ t を指定して求める
		vec3 at(float t) const { return m_origin +t * m_direction; }

	private:
		vec3 m_origin;     // 始点
		vec3 m_direction;  // 方向（非正規化）
	};


	/*-------------------------------------------------------------------------*/
	// カメラ：
	// lookfrom：
	// lookat：
	/*-------------------------------------------------------------------------*/
	class Camera {
	public:
		Camera() {}
		Camera(const vec3& u, const vec3& v, const vec3& w) {
			m_origin = vec3(0);
			m_uvw[0] = u;
			m_uvw[1] = v;
			m_uvw[2] = w;
		}
		Camera(const vec3& lookfrom, const vec3& lookat, const vec3& vup, float vfov, float aspect) {
			vec3 u, v, w;
			float halfH = tanf(radians(vfov) / 2.0f);
			float halfW = aspect * halfH;
			m_origin = lookfrom;
			w = normalize(lookfrom - lookat);
			u = normalize(cross(vup, w));
			v = cross(w, u);
			m_uvw[2] = m_origin - halfW * u - halfH * v - w;
			m_uvw[0] = 2.0f * halfW * u;
			m_uvw[1] = 2.0f * halfH * v;
		}
		// u,v から光線を生成
		Ray getRay(float u, float v) const {
			return Ray(m_origin, m_uvw[2] + m_uvw[0] * u + m_uvw[1] * v - m_origin);
		}
	private:
		vec3 m_origin;  // 位置
		vec3 m_uvw[3];  // 直交基底ベクトル
	};



	/*-------------------------------------------------------------------------*/
	// テクスチャ: 反射率(albedo)を格納した画像データを扱う.
	// u, v: テクスチャ座標
	// p: 対象ピクセルの位置情報
	// value 関数では単にカラー（反射率）を返す.
	/*-------------------------------------------------------------------------*/
	class Texture;
	typedef std::shared_ptr<Texture> TexturePtr;

	class Texture {
	public:
		virtual vec3 value(float u, float v, const vec3& p) const = 0;
	};

	
	/*-------------- (手続き型)単色テクスチャ --------------*/
	class ColorTexture : public Texture {
	public:
		ColorTexture(const vec3& c)
			: m_color(c) {
		}
		virtual vec3 value(float u, float v, const vec3& p) const override {
			return m_color;
		}
	private:
		vec3 m_color;
	};

	/*------------ (手続き型)格子模様テクスチャ ------------*/
	class CheckerTexture : public Texture {
	public:
		// ColorTexture t0, t1 を２つ設定.
		CheckerTexture(const TexturePtr& t0, const TexturePtr& t1, float freq)
			: m_odd(t0)
			, m_even(t1)
			, m_freq(freq) { // 周波数.縞模様の間隔を調整.
		}
		virtual vec3 value(float u, float v, const vec3& p)const override {
			// sinsin  関数を使って交互に格子を描く.
			float sines = sinf(m_freq * p.getX()) * sinf(m_freq * p.getY()) * sinf(m_freq * p.getZ());
			if (sines < 0) {
				return m_odd->value(u, v, p);
			}
			else {
				return m_even->value(u, v, p);
			}
		}
	private:
		TexturePtr m_odd;
		TexturePtr m_even;
		float m_freq;
	};

	/*------------------- 画像テクスチャ -------------------*/
	class ImageTexture : public Texture {
	public:
		ImageTexture(const char* name) {
			int nn;
			// 画像データをファイルから読み込む. stb_image.h を使う．
			m_texels = stbi_load(name, &m_width, &m_height, &nn, 0);
		}
		// 読み込んだデータのメモリを解放
		virtual ~ImageTexture() {
			stbi_image_free(m_texels);
		}
		virtual vec3 value(float u, float v, const vec3& p) const override {
			int i = (u)* m_width;
			int j = (1 - v) * m_height - 0.001;

			// テクスチャ画像からカラーをサンプリング
			return sample(i, j);
		}

		vec3 sample(int u, int v) const
		{
			u = u < 0 ? 0 : u >= m_width ? m_width - 1 : u;
			v = v < 0 ? 0 : v >= m_height ? m_height - 1 : v;
			return vec3(
				int(m_texels[3 * u + 3 * m_width * v]) / 255.0,
				int(m_texels[3 * u + 3 * m_width * v + 1]) / 255.0,
				int(m_texels[3 * u + 3 * m_width * v + 2]) / 255.0);
		}

	private:
		int m_width;
		int m_height;
		unsigned char* m_texels;
	};


	/*--------- 方向を中心とした方向を無作為に作成 ---------*/
	class ONB {
	public:
		ONB() {}
		inline vec3& operator[](int i) {
			return m_axis[i];
		}
		inline const vec3& operator[](int i) const {
			return m_axis[i];
		}
		const vec3& u() const {
			return m_axis[0];
		}
		const vec3& v() const {
			return m_axis[1];
		}
		const vec3& w() const {
			return m_axis[2];
		}
		vec3 local(float a, float b, float c) const {
			return a * u() + b * v() + c * w();
		}
		vec3 local(const vec3& a) const {
			return a.getX() * u() + a.getY() * v() + a.getZ() * w();
		}
		void build_from_w(const vec3& n) {
			m_axis[2] = normalize(n);
			vec3 a;
			if (fabs(w().getX()) > 0.9) {
				a = vec3(0, 1, 0);
			}
			else {
				a = vec3(1, 0, 0);
			}
			m_axis[1] = normalize(cross(w(), a));
			m_axis[0] = cross(w(), v());
		}

	private:
		vec3 m_axis[3];
	};



} // namespace rayt
