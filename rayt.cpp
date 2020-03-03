/*------------------------*/
/*        rayt.cpp        */
/*------------------------*/
#include "rayt.h"


namespace rayt {

	class Shape;
	class Material;
	typedef std::shared_ptr<Shape> ShapePtr;
	typedef std::shared_ptr<Material> MaterialPtr;

/*-------------------------------------------------------------------------------*/
// 物体に衝突したときの情報を格納するクラス
// t: 光線のパラメータ
// p: 衝突した位置
// n: 衝突した位置の法線
// u, v: 衝突したテクスチャ座標
/*-------------------------------------------------------------------------------*/
class HitRec {
	public:
		float t;
		float u;
		float v;
		vec3 p;
		vec3 n;
		MaterialPtr mat;
};

/*--------------------------------------------------------------------------------*/
// scatter 関数は散乱をシミュレーション
// 散乱後の光の向きと，反射率を ScatterRec クラスに格納
/*--------------------------------------------------------------------------------*/
class ScatterRec {
public:
	Ray ray;
	vec3 albedo;
	float pdf_value;
};


/*--------------------------------------------------------------------------------*/
// 材質クラス
/*--------------------------------------------------------------------------------*/
class Material {
public:
	virtual bool scatter(const Ray& r, const HitRec& hrec, ScatterRec& srec) const = 0;
	virtual float scattering_pdf(const Ray& r, const HitRec& hrec) const { return 0; }
	// 発光色を返す仮想関数を追加し，発光する材質はオーバーライド
	// 既存の材質クラスには変更を加えないように純粋仮想関数ではなくて黒を返す
	virtual vec3 emitted(const Ray& r, const HitRec& hrec) const { return vec3(0); }
};

/*-------------------------------------------------------------------------------*/
// ランバート...アルベドを指定. HitRec に材質を追加
// ray   : 散乱後の新しい光線
// albedo: 反射率
/*-------------------------------------------------------------------------------*/
class Lambertian : public Material {
public:
	Lambertian(const TexturePtr& a)
		: m_albedo(a) {
	}
	virtual bool scatter(const Ray& r, const HitRec& hrec, ScatterRec& srec) const override {
		ONB onb;
		onb.build_from_w(hrec.n);
		vec3 direction = onb.local(random_cosin_sirection());
		srec.ray = Ray(hrec.p, normalize(direction));
		srec.albedo = m_albedo->value(hrec.u, hrec.v, hrec.p);
		srec.pdf_value = dot(onb.w(), srec.ray.direction()) / PI;
		return true;
	};

	virtual float scattering_pdf(const Ray& r, const HitRec& hrec) const override {
		return std::max(dot(hrec.n, normalize(r.direction())), 0.0f) / PI;
	}
private:
	TexturePtr m_albedo;
};

/*-------------------------------------------------------------------------------*/
// 金属...鏡面反射
// fuzz: 反射ベクトルのずらし具合
/*-------------------------------------------------------------------------------*/
class Metal : public Material {
public:
	Metal(const TexturePtr& a, float fuzz)
		: m_albedo(a)
		, m_fuzz(fuzz) {
	}
	virtual bool scatter(const Ray& r, const HitRec& hrec, ScatterRec& srec) const override {
		vec3 reflected = reflect(normalize(r.direction()), hrec.n);
		// 通常通り反射ベクトルを計算したあとに，単位球から無作為に点を生成し，
		// それに fuzz を乗算して，計算した反射ベクトルに加算
		reflected += m_fuzz * random_in_unit_sphere();
		srec.ray = Ray(hrec.p, reflected);
		srec.albedo = m_albedo->value(hrec.u, hrec.v, hrec.p);
		return dot(srec.ray.direction(), hrec.n) > 0;
	}
private:
	TexturePtr m_albedo;
	float m_fuzz;
};


/*-------------------------------------------------------------------------------*/
// 誘導体...屈折する素材
// m_ri: 屈折率.屈折ベクトルは物体の内部に入社するときと、内部から外部に
//       出射するときとでは屈折率が反転するので、内積を計算して判定.
// cosine: 入射角.全反射しなければ、近似式(shlick)を使ってフレネル反射率を求める.
// ※基本的に空気中からある媒質に入ることを想定.空気の屈折率は１．
/*-------------------------------------------------------------------------------*/
class Dielectric : public Material {
public:
	Dielectric(float ri)
		: m_ri(ri) {
	}

	virtual bool scatter(const Ray& r, const HitRec& hrec, ScatterRec& srec) const override {
		vec3 outward_normal;
		vec3 reflected = reflect(r.direction(), hrec.n);
		float ni_over_nt;
		float reflect_prob;
		float cosine;
		if (dot(r.direction(), hrec.n) > 0) {
			outward_normal = -hrec.n;
			ni_over_nt = m_ri;
			cosine = m_ri * dot(r.direction(), hrec.n) / length(r.direction());
		}
		else {
			outward_normal = hrec.n;
			ni_over_nt = recip(m_ri);
			cosine = -dot(r.direction(), hrec.n) / length(r.direction());
		}

		srec.albedo = vec3(1);

		vec3 refracted;
		if (refract(-r.direction(), outward_normal, ni_over_nt, refracted)) {
			reflect_prob = schlick(cosine, m_ri);
		}
		else {
			reflect_prob = 1;
		}

		if (drand48() < reflect_prob) {
			srec.ray = Ray(hrec.p, reflected);
		}
		else {
			srec.ray = Ray(hrec.p, refracted);
		}

		return true;
	}

private:
	float m_ri;
};


/*-------------------------------------------------------------------------------*/
// 発光...発光する素材.発光色はテクスチャで設定
/*-------------------------------------------------------------------------------*/
class DiffuseLight : public Material {
public:
	DiffuseLight(const TexturePtr& emit)
		: m_emit(emit) {
	}
	// 散乱はしないため，scatter 関数は何もせずに false を返す．
	virtual bool scatter(const Ray& r, const HitRec& hrec, ScatterRec& srec) const override {
		return false;
	}

	virtual vec3 emitted(const Ray& r, const HitRec& hrec) const override {
		return m_emit->value(hrec.u, hrec.v, hrec.p);
	}

private:
	TexturePtr m_emit;
};


/*-------------------------------------------------------------------------------*/
// 物体の抽象クラス. 衝突関数が仮想関数で定義されている．
// t0 と t1 は光線の衝突範囲
// add 関数で物体を追加でき. 物体は std::shared_ptr<Shape> で渡す
/*-------------------------------------------------------------------------------*/
class Shape {
public:
	virtual bool hit(const Ray& r, float t0, float t1, HitRec& hrec) const = 0;
};


/* Shapeを継承することで、物体リストも単体の物体として動作するようになる */
class ShapeList : public Shape {
public:
	ShapeList() {}

	void add(const ShapePtr& shape) {
		m_list.push_back(shape);
	}

	/* override は仮想関数をオーバーライドすることをコンパイラに知らせるキーワード */
	virtual bool hit(const Ray& r, float t0, float t1, HitRec& hrec) const override {
		HitRec temp_rec;
		/* 何にも衝突しない */
		bool hit_anything = false;
		float closest_so_far = t1;
		for (auto& p : m_list) {
			/* 何かに衝突した */
			if (p->hit(r, t0, closest_so_far, temp_rec)) {
				hit_anything = true;
				closest_so_far = temp_rec.t;
				hrec = temp_rec;
			}
		}
		return hit_anything;
	}
private:
	std::vector<ShapePtr> m_list;
};


class Sphere : public Shape {
public:
	Sphere() {}
	Sphere(const vec3& c, float r, const MaterialPtr& mat)
		: m_center(c)
		, m_radius(r)
		, m_material(mat) {
	}

	virtual bool hit(const Ray& r, float t0, float t1, HitRec& hrec) const override {
		vec3 oc = r.origin() - m_center;
		float a = dot(r.direction(), r.direction());
		float b = 2.0f * dot(oc, r.direction());
		float c = dot(oc, oc) - pow2(m_radius);
		float D = b * b - 4 * a * c;
		if (D > 0) {
			float root = sqrtf(D);
			float temp = (-b - root) / (2.0f * a);
			if (temp < t1 && temp > t0) {
				hrec.t = temp;
				hrec.p = r.at(hrec.t);
				hrec.n = (hrec.p - m_center) / m_radius;
				hrec.mat = m_material;
				get_sphere_uv(hrec.n, hrec.u, hrec.v);
				return true;
			}
			temp = (-b + root) / (2.0f * a);
			if (temp < t1 && temp > t0) {
				hrec.t = temp;
				hrec.p = r.at(hrec.t);
				hrec.n = (hrec.p - m_center) / m_radius;
				hrec.mat = m_material;
				get_sphere_uv(hrec.n, hrec.u, hrec.v);
				return true;
			}
		}

		return false;
	}

private:
	vec3 m_center;
	float m_radius;
	MaterialPtr m_material;
};


/*---------------------------------------------------------------------------------*/
// 四角形
// x0<x<x1x0<x<x1 かつ y0<y<y1y0<y<y1 なら当たっている(XZ軸，YZ軸も同様)
/*---------------------------------------------------------------------------------*/
class Rect : public Shape {
public:
	enum AxisType {
		kXY = 0,
		kXZ,
		kYZ
	};
	Rect() {}
	Rect(float x0, float x1, float y0, float y1, float k, AxisType axis, const MaterialPtr& m)
		: m_x0(x0)
		, m_x1(x1)
		, m_y0(y0)
		, m_y1(y1)
		, m_k(k)
		, m_axis(axis)
		, m_material(m) {
	}

	// 平行な軸によって参照するベクトルの要素を切り替える．
	virtual bool hit(const Ray& r, float t0, float t1, HitRec& hrec) const override {
		int xi, yi, zi;
		vec3 axis;
		// XY軸，XZ軸，YZ軸全てに対応(指定可能)
		switch (m_axis) {
		case kXY: xi = 0; yi = 1; zi = 2; axis = vec3::zAxis(); break;
		case kXZ: xi = 0; yi = 2; zi = 1; axis = vec3::yAxis(); break;
		case kYZ: xi = 1; yi = 2; zi = 0; axis = vec3::xAxis(); break;
		}
		// t が求まり,光線の方程式から平面上の x と y を求められる．
		float t = (m_k - r.origin()[zi]) / r.direction()[zi];
		if (t < t0 || t > t1) {
			return false;
		}

		float x = r.origin()[xi] + t * r.direction()[xi];
		float y = r.origin()[yi] + t * r.direction()[yi];
		if (x < m_x0 || x > m_x1 || y < m_y0 || y > m_y1) {
			return false;
		}

		hrec.u = (x - m_x0) / (m_x1 - m_x0);
		hrec.v = (y - m_y0) / (m_y1 - m_y0);
		hrec.t = t;
		hrec.mat = m_material;
		hrec.p = r.at(t);
		hrec.n = axis;
		return true;
	}

private:
	float m_x0, m_x1, m_y0, m_y1, m_k;
	AxisType m_axis;
	MaterialPtr m_material;
};


/*---------------------------------------------------------------------------------*/
// 法線反転...光線が当たったら，法線を反対方向に向かせる
/*---------------------------------------------------------------------------------*/
class FlipNormals : public Shape {
public:
	FlipNormals(const ShapePtr& shape)
		: m_shape(shape) {
	}
	// 光線が当たったインスタンスの hit 関数を呼んで法線を反転させる．
	virtual bool hit(const Ray& r, float t0, float t1, HitRec& hrec) const override {
		if (m_shape->hit(r, t0, t1, hrec)) {
			hrec.n = -hrec.n;
			return true;
		}
		else {
			return false;
		}
	}
private:
	ShapePtr m_shape;
};


/*---------------------------------------------------------------------------------*/
// 移動...基本は変換する前の状態で当たり判定を行い，結果に対して「アフィン変換」
/*---------------------------------------------------------------------------------*/
class Translate : public Shape {
public:
	Translate(const ShapePtr& sp, const vec3& displacement)
		: m_shape(sp)
		, m_offset(displacement) {
	}
	virtual bool hit(const Ray& r, float t0, float t1, HitRec& hrec) const override {
		Ray move_r(r.origin() - m_offset, r.direction());
		if (m_shape->hit(move_r, t0, t1, hrec)) {
			hrec.p += m_offset;
			return true;
		}
		else {
			return false;
		}
	}

private:
	ShapePtr m_shape;
	vec3 m_offset;
};


/*---------------------------------------------------------------------------------*/
// 回転...クォータニオンを用いて処理
/*---------------------------------------------------------------------------------*/
class Rotate : public Shape {
public:
	Rotate(const ShapePtr& sp, const vec3& axis, float angle)
		: m_shape(sp)
		, m_quat(Quat::rotation(radians(angle), axis)) {
	}
	virtual bool hit(const Ray& r, float t0, float t1, HitRec& hrec) const override {
		Quat revq = conj(m_quat);
		vec3 origin = rotate(revq, r.origin());
		vec3 direction = rotate(revq, r.direction());
		Ray rot_r(origin, direction);
		if (m_shape->hit(rot_r, t0, t1, hrec)) {
			hrec.p = rotate(m_quat, hrec.p);
			hrec.n = rotate(m_quat, hrec.n);
			return true;
		}
		else {
			return false;
		}
	}
private:
	ShapePtr m_shape;
	Quat m_quat;
};


/*---------------------------------------------------------------------------------*/
// 箱を追加...四角形の組み合わせで実装
/*---------------------------------------------------------------------------------*/
class Box : public Shape {
public:
	Box() {}
	Box(const vec3& p0, const vec3& p1, const MaterialPtr& m)
		: m_p0(p0)
		, m_p1(p1)
		, m_list(std::make_unique<ShapeList>()) {

		ShapeList* l = new ShapeList();
		l->add(std::make_shared<Rect>(
			p0.getX(), p1.getX(), p0.getY(), p1.getY(), p1.getZ(), Rect::kXY, m));
		l->add(std::make_shared<FlipNormals>(std::make_shared<Rect>(
			p0.getX(), p1.getX(), p0.getY(), p1.getY(), p0.getZ(), Rect::kXY, m)));
		l->add(std::make_shared<Rect>(
			p0.getX(), p1.getX(), p0.getZ(), p1.getZ(), p1.getY(), Rect::kXZ, m));
		l->add(std::make_shared<FlipNormals>(std::make_shared<Rect>(
			p0.getX(), p1.getX(), p0.getZ(), p1.getZ(), p0.getY(), Rect::kXZ, m)));
		l->add(std::make_shared<Rect>(
			p0.getY(), p1.getY(), p0.getZ(), p1.getZ(), p1.getX(), Rect::kYZ, m));
		l->add(std::make_shared<FlipNormals>(std::make_shared<Rect>(
			p0.getY(), p1.getY(), p0.getZ(), p1.getZ(), p0.getX(), Rect::kYZ, m)));
		m_list.reset(l);
	}

	virtual bool hit(const Ray& r, float t0, float t1, HitRec& hrec) const override {
		return m_list->hit(r, t0, t1, hrec);
	}

private:
	vec3 m_p0, m_p1;
	std::unique_ptr<ShapeList> m_list;
};


/*---------------------------------------------------------------------------------*/
// ShapeBuilder...物体と法線反転などの追加を行える. すべて自己参照を返す
/*---------------------------------------------------------------------------------*/
class ShapeBuilder {
public:
	ShapeBuilder() {}
	ShapeBuilder(const ShapePtr& sp)
		: m_ptr(sp) {
	}

	ShapeBuilder& reset(const ShapePtr& sp) {
		m_ptr = sp;
		return *this;
	}

	ShapeBuilder& sphere(const vec3& c, float r, const MaterialPtr& m) {
		m_ptr = std::make_shared<Sphere>(c, r, m);
		return *this;
	}

	ShapeBuilder& rect(float x0, float x1, float y0, float y1, float k, Rect::AxisType axis, const MaterialPtr& m) {
		m_ptr = std::make_shared<Rect>(x0, x1, y0, y1, k, axis, m);
		return *this;
	}
	ShapeBuilder& rectXY(float x0, float x1, float y0, float y1, float k, const MaterialPtr& m) {
		m_ptr = std::make_shared<Rect>(x0, x1, y0, y1, k, Rect::kXY, m);
		return *this;
	}
	ShapeBuilder& rectXZ(float x0, float x1, float y0, float y1, float k, const MaterialPtr& m) {
		m_ptr = std::make_shared<Rect>(x0, x1, y0, y1, k, Rect::kXZ, m);
		return *this;
	}
	ShapeBuilder& rectYZ(float x0, float x1, float y0, float y1, float k, const MaterialPtr& m) {
		m_ptr = std::make_shared<Rect>(x0, x1, y0, y1, k, Rect::kYZ, m);
		return *this;
	}

	ShapeBuilder& rect(const vec3& p0, const vec3& p1, float k, Rect::AxisType axis, const MaterialPtr& m) {
		switch (axis) {
		case Rect::kXY:
			m_ptr = std::make_shared<Rect>(
				p0.getX(), p1.getX(), p0.getY(), p1.getY(), k, axis, m);
			break;
		case Rect::kXZ:
			m_ptr = std::make_shared<Rect>(
				p0.getX(), p1.getX(), p0.getZ(), p1.getZ(), k, axis, m);
			break;
		case Rect::kYZ:
			m_ptr = std::make_shared<Rect>(
				p0.getY(), p1.getY(), p0.getZ(), p1.getZ(), k, axis, m);
			break;
		}
		return *this;
	}
	ShapeBuilder& rectXY(const vec3& p0, const vec3& p1, float k, const MaterialPtr& m) {
		return rect(p0, p1, k, Rect::kXY, m);
	}
	ShapeBuilder& rectXZ(const vec3& p0, const vec3& p1, float k, const MaterialPtr& m) {
		return rect(p0, p1, k, Rect::kXZ, m);
	}
	ShapeBuilder& rectYZ(const vec3& p0, const vec3& p1, float k, const MaterialPtr& m) {
		return rect(p0, p1, k, Rect::kYZ, m);
	}

	ShapeBuilder& box(const vec3& p0, const vec3& p1, const MaterialPtr& m) {
		m_ptr = std::make_shared<Box>(p0, p1, m);
		return *this;
	}

	ShapeBuilder& flip() {
		m_ptr = std::make_shared<FlipNormals>(m_ptr);
		return *this;
	}

	ShapeBuilder& translate(const vec3& t) {
		m_ptr = std::make_shared<Translate>(m_ptr, t);
		return *this;
	}

	ShapeBuilder& rotate(const vec3& axis, float angle) {
		m_ptr = std::make_shared<Rotate>(m_ptr, axis, angle);
		return *this;
	}

	const ShapePtr& get() const { return m_ptr; }

private:
	ShapePtr m_ptr;
};



/*---------------------------------------------------------------------------------*/
// レンダリングするシーンのカラーなどの設定
// 生成する球体を追加、材質の設定
/*---------------------------------------------------------------------------------*/
class Scene {
public:
	Scene(int width, int height, int samples)
		: m_image(std::make_unique<Image>(width, height))
		, m_backColor(0.2f)
		, m_samples(samples) {
	}



	/*  レンダリングするときに一度呼ばれる関数で */
	/*  カメラや物体の生成などの初期化を行う     */
	void build() {
		
		m_backColor = vec3(0);

		// Camera

		vec3 lookfrom(278, 278, -800);
		vec3 lookat(278, 278, 0);
		vec3 vup(0, 1, 0);
		float aspect = float(m_image->width()) / float(m_image->height());
		m_camera = std::make_unique<Camera>(lookfrom, lookat, vup, 40, aspect);

		// Shapes

		MaterialPtr red = std::make_shared<Lambertian>(
			std::make_shared<ColorTexture>(vec3(0.65f, 0.05f, 0.05f)));
		MaterialPtr white = std::make_shared<Lambertian>(
			std::make_shared<ColorTexture>(vec3(0.73f)));
		MaterialPtr green = std::make_shared<Lambertian>(
			std::make_shared<ColorTexture>(vec3(0.12f, 0.45f, 0.15f)));
		MaterialPtr light = std::make_shared<DiffuseLight>(
			std::make_shared<ColorTexture>(vec3(15.0f)));
			
		ShapeList* world = new ShapeList();
		ShapeBuilder builder;
		world->add(builder.rectYZ(0, 555, 0, 555, 555, green).flip().get());
		world->add(builder.rectYZ(0, 555, 0, 555, 0, red).get());
		world->add(builder.rectXZ(213, 343, 227, 332, 554, light).get());
		world->add(builder.rectXZ(0, 555, 0, 555, 555, white).flip().get());
		world->add(builder.rectXZ(0, 555, 0, 555, 0, white).get());
		world->add(builder.rectXY(0, 555, 0, 555, 555, white).flip().get());
		world->add(builder.box(vec3(0), vec3(165), white)
			.rotate(vec3::yAxis(), -18)
			.translate(vec3(130, 0, 65))
			.get());
		world->add(builder.box(vec3(0), vec3(165, 330, 165), white)
			.rotate(vec3::yAxis(), 15)
			.translate(vec3(265, 0, 295))
			.get());

		m_world.reset(world);
	}


/*---------------------------------------------------------------------------------*/
// カラーの設定. 反射するたび再帰的に呼び出させる
// depth: ある程度の反射回数以上は計算の無駄なので、特定の反射回数で打ち切る
/*---------------------------------------------------------------------------------*/
	vec3 color(const rayt::Ray& r, const Shape* world, int depth) {
		HitRec hrec;
		if (world->hit(r, 0.001f, FLT_MAX, hrec)) {
		//　反射した光に発光を加える．
			vec3 emitted = hrec.mat->emitted(r, hrec);
			ScatterRec srec;

		/* 散乱させて，次の方向から受け取った色と反射率を乗算 */
		/*if (hrec.mat->scatter(r, hrec, srec) && depth < MAX_DEPTH) {
			return mulPerElem(srec.albedo, color(srec.ray, world, depth+1));
		}*/

			if (depth < MAX_DEPTH && hrec.mat->scatter(r, hrec, srec) && srec.pdf_value > 0) {

				vec3 on_light = vec3(213 + drand48() * (343 - 213), 554, 227 + drand48() * (332 - 227));
				vec3 to_light = on_light - hrec.p;
				float distance_squared = lengthSqr(to_light);
				to_light = normalize(to_light);
				if (dot(to_light, hrec.n) < 0) {
					return emitted;
				}

				float light_area = (343 - 213) * (332 - 227);
				float light_cosine = fabs(to_light.getY());
				if (light_cosine < 0.000001) {
					return emitted;
				}
				srec.pdf_value = distance_squared / (light_cosine * light_area);
				srec.ray = Ray(hrec.p, to_light);

				float spdf_value = hrec.mat->scattering_pdf(srec.ray, hrec);
				vec3 albedo = srec.albedo * spdf_value;
				return emitted + mulPerElem(albedo, color(srec.ray, world, depth + 1)) / srec.pdf_value;
			}
			else {
				return emitted;
			}
		}
		return background(r.direction());
	}

/* 単色用の背景色 */
vec3 background(const vec3& d) const {
	return m_backColor;
}

/* 背景色としてのグラデーション */
vec3 backgroundSky(const vec3& d) const {
	vec3 v = normalize(d);
	float t = 0.5f * (v.getY() + 1.0f);
	return lerp(t, vec3(1), vec3(0.5f, 0.7f, 1.0f));
}

		/* レンダリングを行う */
		void render() {
			/* 初期化 */
			build();

			int nx = m_image->width();
			int ny = m_image->height();

			/* 並列処理のフレームワークである OpenMP を使う */
			/* OpenMP はコード中にディレクティブを挿入する  */
			/* 簡単に並列処理を実現することがでる           */
			#pragma omp parallel for schedule(dynamic, 1) num_threads(NUM_THREAD)
			for (int j = 0; j < ny; ++j) {
				std::cerr << "Rendering (y = " << j << ") " << (100.0 * j / (ny - 1)) << "%" << std::endl;
				for (int i = 0; i < nx; ++i) {
					vec3 c(0);
					for (int s = 0; s < m_samples; s++) {
						float u = (float(i) + drand48()) / float(nx);
						float v = (float(j) + drand48()) / float(ny);
						Ray r = m_camera->getRay(u, v);
						c += color(r, m_world.get(), 0);
					}
					c /= m_samples;
					m_image->write(i,( ny - j - 1), c.getX(), c.getY(), c.getZ());
				}
			}

			stbi_write_bmp("renderSample.jpg", nx, ny, sizeof(Image::rgb), m_image->pixels());
		}

	private:
		std::unique_ptr<Camera> m_camera;
		std::unique_ptr<Image> m_image;
		std::unique_ptr<Shape> m_world;
		vec3 m_backColor;
		int m_samples;
	};

}// namespace rayt


int main()
{
	int nx = 200;
	int ny = 200;
	int ns = 500; // サンプリング数
	std::unique_ptr<rayt::Scene> scene(std::make_unique<rayt::Scene>(nx, ny, ns));
	scene->render();
	return 0;
}