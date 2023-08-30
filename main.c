#define LINEARLIB_IMPLEMENTATION
#include "linear.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <assert.h>

#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>

typedef vec3_t colour_t;
typedef vec3_t point_t;

#define IMAGE_WIDTH  800
#define ASPECT_RATIO 16.0 / 9.0

#define PPM_PATH "output.ppm"

FILE *ppm_out;

#define CROSS(a, b)       ll_vec3_cross3fv(a, b)
#define DOT(a, b)         ll_vec3_dot3fv(a, b)
#define SUB(a, b)         ll_vec3_sub3fv(a, b)
#define ADD(a, b)         ll_vec3_add3fv(a, b)
#define MUL(a, b)         ll_vec3_mul3fv(a, b)
#define COLOUR(r, g, b)   ll_vec3_create3f(r, g, b)
#define POINT(x, y, z)    ll_vec3_create3f(x, y, z)
#define VEC3(x, y, z)     ll_vec3_create3f(x, y, z)
#define LENGTH_SQUARED(a) ll_vec3_length_squared3fv(a)

static inline float
degrees_to_radians(float degrees)
{
	return degrees * M_PI / 180.0;
}

static inline float
random_float(void)
{
	return rand() / (RAND_MAX + 1.0);
}

static inline float
random_float2f(float min, float max)
{
	return min + (max-min) * random_float();
}

static inline int
random_int2i(int min, int max)
{
	return (int) (random_float2f(min, max + 1));
}

#define RANDOMVEC3() VEC3(random_float(), random_float(), random_float())
#define RANDOMVEC32f(min, max) VEC3(random_float2f(min, max), random_float2f(min, max), \
				    random_float2f(min, max))

static inline vec3_t
random_in_unit_sphere(void)
{
	while (true) {
		vec3_t p = RANDOMVEC32f(-1, 1);
		if (LENGTH_SQUARED(p) < 1)
			return p;
	}
}

static inline vec3_t
random_unit_vector(void)
{
	return ll_vec3_normalise3fv(random_in_unit_sphere());
}

static inline vec3_t
random_in_unit_disk(void)
{
	while (true) {
		vec3_t p = ll_vec3_create3f(random_float2f(-1, 1), random_float2f(-1, 1), 0);
		if (LENGTH_SQUARED(p) < 1)
			return p;
	}
}

static inline vec3_t
random_on_hemisphere(vec3_t normal)
{
	vec3_t on_unit_sphere = random_unit_vector();
	if (DOT(on_unit_sphere, normal) > 0.0)
		return on_unit_sphere;
	return ll_vec3_mul1f(on_unit_sphere, -1.0);
}

static inline vec3_t
reflect(vec3_t v, vec3_t n)
{
	return SUB(v, ll_vec3_mul1f(ll_vec3_mul1f(n, DOT(v, n)), 2));
}

static inline vec3_t
refract(vec3_t uv, vec3_t n, float etai_over_etat)
{
	float cos_theta = fmin(DOT(ll_vec3_mul1f(uv, -1), n), 1.0);
	vec3_t r_out_perp = ll_vec3_mul1f(ADD(ll_vec3_mul1f(n, cos_theta), uv), etai_over_etat);
	vec3_t r_out_parallel = ll_vec3_mul1f(n, -sqrt(fabs(1.0 - LENGTH_SQUARED(r_out_perp))));
	return ADD(r_out_perp, r_out_parallel);
}

static inline float
linear_to_gamma(float linear_component)
{
	return sqrt(linear_component);
}

static inline bool
near_zero(vec3_t v)
{
	float s = 1e-8;
	return (fabs(v.x) < s) && (fabs(v.y) < s) && (fabs(v.z) < s);
}

static point_t
defocus_disk_sample(void);

typedef struct interval_t {
	float min, max;
} interval_t;

static bool
interval_contains(interval_t interval, float x);

static bool
interval_surrounds(interval_t interval, float x);

static float
interval_clamp(interval_t interval, float x);

static float
interval_size(interval_t interval);

static interval_t
interval_expand(interval_t interval, float delta);

#define INTERVAL2f(a, b) (interval_t) { a, b }


const static interval_t empty    = INTERVAL2f(+INFINITY, -INFINITY);
const static interval_t universe = INTERVAL2f(-INFINITY, +INFINITY);

typedef struct ray_t {
	point_t origin;
	vec3_t direction;
	float time;
} ray_t;

#define RAY(origin, direction) (ray_t) { origin, direction, 0.0 }
#define RAY1f(origin, direction, time) (ray_t) { origin, direction, time }
#define RAY_AT(ray, t) ADD(ray.origin, ll_vec3_mul1f(ray.direction, t))

typedef struct aabb_t {
	interval_t x, y, z;
} aabb_t;

#define INTERVAL2v(a, b) INTERVAL2f(fmin(a.min, b.min), fmax(a.max, b.max))

static aabb_t
aabb_empty(void);

static aabb_t
aabb2v(point_t a, point_t b);

static aabb_t
aabb3i(interval_t x, interval_t y, interval_t z);

static aabb_t
aabb2bbox(aabb_t a, aabb_t b);

static interval_t
aabb_axis(aabb_t aabb, int n);

static bool
aabb_hit(aabb_t aabb, ray_t ray, interval_t ray_t);

typedef struct solid_colour_t {
	colour_t colour;
} solid_colour_t;

typedef struct checkered_t {
	float inv_scale;
	struct texture_t *even;
	struct texture_t *odd;
} checkered_t;

typedef struct image_t {
	SDL_Surface *surface;
} image_t;

typedef struct noise_t {
	int point_count;
	vec3_t *ran_vec;
	int *perm_x;
	int *perm_y;
	int *perm_z;
	float scale;
} noise_t;

typedef struct texture_t {
	enum {
		TEXTURE_SOLID_COLOUR,
		TEXTURE_CHECKERED,
		TEXTURE_IMAGE,
		TEXTURE_NOISE,
		TEXTURE_COUNT,
	} type;

	union {
		struct solid_colour_t solid_colour;
		struct checkered_t checkered;
		struct image_t image;
		struct noise_t noise;
	} surface;

	colour_t (*value)(struct texture_t *texture, float u, float v, point_t p);
} texture_t;

static texture_t *
solid_colour_create3f(float r, float g, float b);

static colour_t
solid_colour_value(texture_t *texture, float u, float v, point_t p);

static texture_t *
checkered_create(float scale, texture_t *even, texture_t *odd);

static texture_t *
checkered_create2v(float scale, colour_t c1, colour_t c2);

static colour_t
checkered_value(texture_t *texture, float u, float v, point_t p);

static texture_t *
image_create(const char *pathname);

static colour_t
image_value(texture_t *texture, float u, float v, point_t p);

static texture_t *
noise_create(float sc);

static colour_t
noise_value(texture_t *texture, float u, float v, point_t p);

typedef struct hit_record_t hit_record_t;

typedef enum mat_t {
	MATERIAL_LAMBERTIAN,
	MATERIAL_METAL,
	MATERIAL_DIELECTRIC,
	MATERIAL_COUNT,
} mat_t;

typedef struct lambertian_t {
	struct texture_t *albedo;
} lambertian_t;

typedef struct metal_t {
	colour_t albedo;
	float fuzz;
} metal_t;

typedef struct dielectric_t {
	float ir;
} dielectric_t;

typedef struct material_t {
	mat_t type;
	union {
		lambertian_t lambertian;
		metal_t metal;
		dielectric_t dielectric;
	} surface;
	bool (*scatter)(struct material_t *material, ray_t ray, struct hit_record_t *hit_record,
			colour_t *attenuation, ray_t *scattered);
} material_t;

static material_t *
lambertian_create(texture_t *texture);

static bool
lambertian_scatter(struct material_t *material, ray_t ray, struct hit_record_t *hit_record,
		   colour_t *attenuation, ray_t *scattered);

static bool
metal_scatter(struct material_t *material, ray_t ray, struct hit_record_t *hit_record,
		   colour_t *attenuation, ray_t *scattered);

static bool
dielectric_scatter(struct material_t *material, ray_t ray, struct hit_record_t *hit_record,
		   colour_t *attenuation, ray_t *scattered);

static float
reflectance(float cosine, float ref_idx);

#define LAMBERTIAN_MATERIAL(colour) (material_t) { .type = MATERIAL_LAMBERTIAN,	\
			.surface.lambertian = (lambertian_t) { colour }, .scatter = lambertian_scatter}

#define METAL_MATERIAL(colour, f) (material_t) { .type = MATERIAL_METAL,	\
			.surface.metal = (metal_t) { colour, f < 1 ? f : 1 }, .scatter = metal_scatter}

#define DIELECTRIC_MATERIAL(ir) (material_t) { .type = MATERIAL_DIELECTRIC,	\
			.surface.dielectric = (dielectric_t) { ir }, .scatter = dielectric_scatter} 


typedef struct hit_record_t {
	point_t p;
	vec3_t normal;
	material_t *mat;
	float t;
	float u;
	float v;
	bool front_face;
} hit_record_t;

hit_record_t hit_record;

typedef enum obj_t {
	OBJECT_SPHERE,
	OBJECT_BVH,
	OBJECT_COUNT,
} obj_t;

typedef struct sphere_t {
	point_t center1;
	float radius;
	material_t *mat;
	bool is_moving;
	vec3_t center_vec;
	aabb_t bbox;
} sphere_t;

typedef struct bvh_node_t {
	struct hittable_t *left;
	struct hittable_t *right;
	aabb_t bbox;
} bvh_node_t;

typedef struct hittable_t {
	obj_t type;
	union {
		struct sphere_t sphere;
		struct bvh_node_t bvh_node;
	} object;
	bool (*hit)(ray_t ray, struct hittable_t *hittable, interval_t interval,
		    hit_record_t *hit_record);
	aabb_t (*bounding_box)(struct hittable_t *hittable);
} hittable_t;

static hittable_t *
sphere_create(point_t center1, float radius, material_t *mat);

static hittable_t *
sphere_move_create(point_t center1, point_t center2, float radius, material_t *mat);

static bool
sphere_hit(ray_t ray, hittable_t *sphere, interval_t ray_t, hit_record_t *hit_record);

static point_t
sphere_center(sphere_t sphere, float time)
{
	return ADD(sphere.center1, ll_vec3_mul1f(sphere.center_vec, time));
}

static void
get_sphere_uv(point_t p, float *u, float *v)
{
	float theta = acos(-p.y);
	float phi = atan2(-p.z, p.x) + M_PI;

	*u = phi / (2*M_PI);
	*v = theta / M_PI;
}

static aabb_t
sphere_bounding_box(struct hittable_t *hittable);

#define SPHERE_AABB(center1, radius) aabb2v(SUB(center1, VEC3(radius, radius, radius)), \
					   ADD(center1, VEC3(radius, radius, radius)))

#define SPHERE_MOVE_AABB(center1, center2, radius) aabb2bbox(SPHERE_AABB(center1, radius), SPHERE_AABB(center2, radius))

#define SPHERE(center1, radius, mat) (hittable_t) { OBJECT_SPHERE,	\
			.object.sphere = (sphere_t) { center1, radius, mat, false, \
						      center1, SPHERE_AABB(center1, radius) }, \
			.hit = sphere_hit, .bounding_box = sphere_bounding_box }

#define SPHERE_MOVE(center1, center2, radius, mat) (hittable_t) { OBJECT_SPHERE, \
			.object.sphere = (sphere_t) { center1, radius, mat, true, \
						      SUB(center2, center1), SPHERE_MOVE_AABB(center1, center2, radius) }, \
			.hit = sphere_hit, .bounding_box = sphere_bounding_box } 

typedef struct hit_list_t hit_list_t;

static hittable_t *
bvh_node_create(struct hit_list_t *list, size_t start, size_t end);

static bool
bvh_node_hit(ray_t ray, hittable_t *bvh_node, interval_t ray_t, hit_record_t *hit_record);

static aabb_t
bvh_node_bounding_box(struct hittable_t *hittable);

static int
box_compare(struct hittable_t *a, struct hittable_t *b, int index);

static int
box_x_compare(struct hittable_t *a, struct hittable_t *b);

static int
box_y_compare(struct hittable_t *a, struct hittable_t *b);

static int
box_z_compare(struct hittable_t *a, struct hittable_t *b);

typedef struct hit_list_t {
	size_t size;
	size_t capacity;
	hittable_t **list;
	aabb_t bbox;
} hit_list_t;

hit_list_t world;

#define HIT_LIST_DEFAULT_CAPACITY 8

static void
hit_list_push_back(hit_list_t *list, hittable_t *hittable);
static bool
hit_list_hit(hit_list_t *list, ray_t ray, interval_t ray_t);
static hit_list_t
hit_list_copy(hit_list_t *list);
static aabb_t
hit_list_bounding_box(hit_list_t *list);

static void
set_face_normal(ray_t ray, hit_record_t *rec, const vec3_t *outward_normal);

static void
write_colour(colour_t colour, int samples_per_pixel);

static float
hit_sphere(point_t center, float radius);


struct {
	double aspect_ratio;
	int image_width;
	int image_height;
	point_t center;
	point_t pixel00_loc;
	vec3_t pixel_delta_u;
	vec3_t pixel_delta_v;
	int samples_per_pixel;
	int max_depth;
	
	float vfov;
	point_t lookfrom;
	point_t lookat;
	vec3_t vup;
	vec3_t u, v, w;
	vec3_t defocus_disk_u;
	vec3_t defocus_disk_v;

	float defocus_angle;
	float focus_dist;
} camera;

static void
camera_render(hit_list_t *list);

static void
camera_initialize(void);

static colour_t
camera_ray_colour(hit_list_t *list, ray_t ray, int depth);

static ray_t
camera_get_ray(int i, int j);

static vec3_t
camera_pixel_sample_square(void);

static int
internal_box_x_compare(const void *a, const void *b)
{
	hittable_t *a_ = *(hittable_t **) a;
	hittable_t *b_ = *(hittable_t **) b;
	return box_x_compare(a_, b_);
}

static int
internal_box_y_compare(const void *a, const void *b)
{
	hittable_t *a_ = *(hittable_t **) a;
	hittable_t *b_ = *(hittable_t **) b;
	return box_y_compare(a_, b_);
}

static int
internal_box_z_compare(const void *a, const void *b)
{
	hittable_t *a_ = *(hittable_t **) a;
	hittable_t *b_ = *(hittable_t **) b;
	return box_z_compare(a_, b_);
}

static void
random_spheres(void)
{
	texture_t *checkered = checkered_create2v(0.8, COLOUR(.2, .3, .1), COLOUR(.9, .9, .9));
	material_t *material_ground = lambertian_create(checkered);
	hit_list_push_back(&world, sphere_create(POINT(0, -1000, 0), 1000, material_ground));
	
	//for (int a = -11; a < 11; a++) {
	//	for (int b = -11; b < 11; b++) {
	//		float choose_mat = random_float();
	//		point_t center = POINT(a + 0.9*random_float(), 0.2, b + 0.9*random_float());
	//
	//		if (ll_vec3_length3fv(SUB(center, POINT(4, 0.2, 0))) > 0.9) {
	//			material_t sphere_material;
	//
	//			if (choose_mat < 0.0) {
	//				vec3_t albedo = RANDOMVEC3();
	//				sphere_material = LAMBERTIAN_MATERIAL(albedo);
	//				hit_list_push_back(&world, sphere_create(center, 0.2,
	//							  sphere_material));
	//			} else if (choose_mat < 0.95) {
	//				vec3_t albedo = RANDOMVEC32f(0.5, 1);
	//				float fuzz = random_float2f(0, 0.5);
	//				sphere_material = METAL_MATERIAL(albedo, fuzz);
	//				hit_list_push_back(&world, sphere_create(center, 0.2,
	//							  sphere_material));
	//			} else {
	//				sphere_material = DIELECTRIC_MATERIAL(1.5);
	//				hit_list_push_back(&world, sphere_create(center, 0.2,
	//							  sphere_material));
	//			}
	//		}
	//	}
	//}
	
	//material_t material1 = DIELECTRIC_MATERIAL(1.5);
	//material_t material2 = LAMBERTIAN_MATERIAL(COLOUR(0.4, 0.2, 0.1));
	//material_t material3 = METAL_MATERIAL(COLOUR(0.7, 0.6, 0.5), 0.0);
	//hit_list_push_back(&world, sphere_create(POINT(0, 1, 0), 1.0, material1));
	//hit_list_push_back(&world, sphere_create(POINT(-4, 1, 0), 1.0, material2));
	//hit_list_push_back(&world, sphere_create(POINT(4, 1, 0), 1.0, material3));

	hittable_t *bvh_node = bvh_node_create(&world, 0, world.size);
	world.size = 0;
	hit_list_push_back(&world, bvh_node);

	camera.image_width = IMAGE_WIDTH;
	camera.aspect_ratio = ASPECT_RATIO;
	camera.samples_per_pixel = 500;
	camera.max_depth = 50;
	
	camera.vfov = 20;
	camera.lookfrom = VEC3(13, 2, 3);
	camera.lookat = VEC3(0, 0, 0);
	camera.vup = VEC3(0, 1, 0);
	
	camera.defocus_angle = 0.6;
	camera.focus_dist = 10.0;
	
	camera_render(&world);
}

static void
two_spheres(void)
{
	texture_t *checkered = checkered_create2v(0.8, COLOUR(.2, .3, .1), COLOUR(.9, .9, .9));
	material_t *material = lambertian_create(checkered);

	hit_list_push_back(&world, sphere_create(POINT(0, -10, 0), 10.0, material));
	hit_list_push_back(&world, sphere_create(POINT(0, 10, 0), 10.0, material));

	camera.image_width = IMAGE_WIDTH;
	camera.aspect_ratio = ASPECT_RATIO;
	camera.samples_per_pixel = 500;
	camera.max_depth = 50;
	
	camera.vfov = 20;
	camera.lookfrom = VEC3(13, 2, 3);
	camera.lookat = VEC3(0, 0, 0);
	camera.vup = VEC3(0, 1, 0);
	
	camera.defocus_angle = 0.6;
	camera.focus_dist = 10.0;
	
	camera_render(&world);
}

static void
earth(void)
{
	texture_t *earth = earth = image_create("earthmap.jpg");
	material_t *material = lambertian_create(earth);

	hit_list_push_back(&world, sphere_create(POINT(0, 0, 0), 2.0, material));
	
	camera.image_width = IMAGE_WIDTH;
	camera.aspect_ratio = ASPECT_RATIO;
	camera.samples_per_pixel = 500;
	camera.max_depth = 50;
	
	camera.vfov = 20;
	camera.lookfrom = VEC3(0, 0, 12);
	camera.lookat = VEC3(0, 0, 0);
	camera.vup = VEC3(0, 1, 0);
	
	camera.defocus_angle = 0.0;
	camera.focus_dist = 10.0;
	
	camera_render(&world);
}

static void
two_perlin_spheres(void)
{
	texture_t *perlin = noise_create(4.0);
	material_t *material = lambertian_create(perlin);

	hit_list_push_back(&world, sphere_create(POINT(0, -1000, 0), 1000.0, material));
	hit_list_push_back(&world, sphere_create(POINT(0, 2, 0), 2.0, material));
	
	camera.image_width = IMAGE_WIDTH;
	camera.aspect_ratio = ASPECT_RATIO;
	camera.samples_per_pixel = 50;
	camera.max_depth = 50;
	
	camera.vfov = 20;
	camera.lookfrom = VEC3(13, 2, 3);
	camera.lookat = VEC3(0, 0, 0);
	camera.vup = VEC3(0, 1, 0);
	
	camera.defocus_angle = 0.0;
	camera.focus_dist = 10.0;
	
	camera_render(&world);
}

int
main(int argc, char **argv)
{
	srand(5);
	SDL_Init(SDL_INIT_VIDEO);
	IMG_Init(IMG_INIT_PNG | IMG_INIT_JPG);
	switch (4) {
	case 1:
		random_spheres();
		break;
	case 2:
		two_spheres();
		break;
	case 3:
		earth();
		break;
	case 4:
		two_perlin_spheres();
		break;
	}
	SDL_Quit();
	IMG_Quit();
	return 0;
}

static bool 
interval_contains(interval_t interval, float x)
{
	return interval.min <= x && x <= interval.max;
}

static bool
interval_surrounds(interval_t interval, float x)
{
	return interval.min < x && x < interval.max;
}

static float
interval_clamp(interval_t interval, float x)
{
	if (x < interval.min) return interval.min;
	if (x > interval.max) return interval.max;
	return x;
}

static float
interval_size(interval_t interval)
{
	return interval.max - interval.min;
}

static interval_t
interval_expand(interval_t interval, float delta)
{
	float padding = delta / 2;
	return INTERVAL2f(interval.min - padding, interval.max + padding);
}

static void
write_colour(colour_t colour, int samples_per_pixel)
{
	float r = colour.r;
	float g = colour.g;
	float b = colour.b;

	float scale = 1.0 / (float) samples_per_pixel;
	r *= scale;
	g *= scale;
	b *= scale;

	r = linear_to_gamma(r);
	g = linear_to_gamma(g);
	b = linear_to_gamma(b);

	static const interval_t intensity = INTERVAL2f(0.000, 0.999);
	fprintf(ppm_out, "%d %d %d\n",
		(int) (256 * interval_clamp(intensity, r)),
		(int) (256 * interval_clamp(intensity, g)),
		(int) (256 * interval_clamp(intensity, b)));
}

static void
hit_list_push_back(hit_list_t *list, hittable_t *hittable)
{
	void *new_list;
	if (list->list) {
		if (list->size == list->capacity) {
			new_list = realloc(list->list, sizeof(*list->list) * list->capacity*2);
			if (!new_list) {
				fprintf(stderr, "Failed to resize list!\n");
				exit(EXIT_FAILURE);
			}
			list->list = new_list;
			list->capacity *= 2;
		}
	} else {
		list->size = 0;
		list->capacity = HIT_LIST_DEFAULT_CAPACITY;
		list->list = calloc(list->capacity, sizeof(*list->list));
		if (!list->list) {
			fprintf(stderr, "Failed to allocate list!\n");
			exit(EXIT_FAILURE);
		}
		list->bbox = aabb_empty();
	}

	list->list[list->size] = hittable;
	++list->size;
	list->bbox = aabb2bbox(list->bbox, hittable->bounding_box(hittable));
}

static bool
hit_list_hit(hit_list_t *list, ray_t ray, interval_t ray_t)
{
	size_t i;
	hit_record_t temp_rec;
	bool hit_anything = false;
	float closest_so_far = ray_t.max;
	hittable_t *hittable;

	for (i = 0; i < list->size; i++) {
		hittable = list->list[i];
		if (hittable->hit(ray, hittable, INTERVAL2f(ray_t.min, closest_so_far), &temp_rec)) {
			hit_anything = true;
			closest_so_far = temp_rec.t;
			hit_record = temp_rec;
		}
	}
	
	return hit_anything;
}

static hit_list_t
hit_list_copy(hit_list_t *list)
{
	hit_list_t new_list;
	new_list.capacity = list->size;
	new_list.size = list->size;
	new_list.list = calloc(new_list.capacity, sizeof(*new_list.list));
	if (!new_list.list) {
		fprintf(stderr, "Failed to allocate list!\n");
		exit(EXIT_FAILURE);
	}

	memcpy(new_list.list, list->list, new_list.capacity * sizeof(*new_list.list));
	return new_list;
}

static aabb_t
hit_list_bounding_box(hit_list_t *list)
{
	return list->bbox;
}

static void
set_face_normal(ray_t ray, hit_record_t *rec, const vec3_t *outward_normal)
{
	rec->front_face = DOT(ray.direction, *outward_normal) < 0;
	rec->normal = rec->front_face ? *outward_normal
		: ll_vec3_mul1f(*outward_normal, -1.0);
}

static hittable_t *
bvh_node_create(hit_list_t *list, size_t start, size_t end)
{
	hittable_t *hittable;
	hittable = malloc(sizeof(*hittable));
	if (!hittable) {
		fprintf(stderr, "Failed to allocate bvh node!\n");
		exit(EXIT_FAILURE);
	}

	int axis = random_int2i(0, 2);
	int (*cmp)(const void *, const void *);
	cmp = (axis == 0) ? internal_box_x_compare
		: (axis == 1) ? internal_box_y_compare
		: internal_box_z_compare;

	hittable->type = OBJECT_BVH;

	hittable->object.bvh_node = (bvh_node_t) { NULL, NULL, aabb_empty() };
	hittable->hit = bvh_node_hit;
	hittable->bounding_box = bvh_node_bounding_box;

	hit_list_t new_list = hit_list_copy(list);

	size_t object_span = end - start;
	if (object_span == 1) {
		hittable->object.bvh_node.left = hittable->object.bvh_node.right = new_list.list[start];
	} else if (object_span == 2) {
		if (cmp(new_list.list+start, new_list.list+start+1)) {
			hittable->object.bvh_node.left = new_list.list[start];
			hittable->object.bvh_node.right = new_list.list[start+1];
		} else {
			hittable->object.bvh_node.left = new_list.list[start+1];
			hittable->object.bvh_node.right = new_list.list[start];
		}
	} else {
		int mid = start + object_span / 2;
		qsort(new_list.list + start, object_span, sizeof(*new_list.list), cmp);
		hittable->object.bvh_node.left = bvh_node_create(&new_list, start, mid);
		hittable->object.bvh_node.right = bvh_node_create(&new_list, mid, end);
	}

	hittable->object.bvh_node.bbox = aabb2bbox(hittable->object.bvh_node.left->bounding_box(hittable->object.bvh_node.left),
						   hittable->object.bvh_node.right->bounding_box(hittable->object.bvh_node.right));
	return hittable;
}

static bool
bvh_node_hit(ray_t ray, hittable_t *bvh_node, interval_t ray_t, hit_record_t *hit_record)
{
	assert(bvh_node->type == OBJECT_BVH);
	bvh_node_t bvh = bvh_node->object.bvh_node;
	if (!aabb_hit(bvh.bbox, ray, ray_t))
		return false;

	bool hit_left = bvh.left->hit(ray, bvh.left, ray_t, hit_record);
	bool hit_right = bvh.right->hit(ray, bvh.right, INTERVAL2f(ray_t.min, hit_left ? hit_record->t
								    : ray_t.max), hit_record);
	return hit_left || hit_right;
}

static aabb_t
bvh_node_bounding_box(struct hittable_t *hittable)
{
	assert(hittable->type == OBJECT_BVH);
	bvh_node_t bvh = hittable->object.bvh_node;
	return bvh.bbox;
}

static int
box_compare(struct hittable_t *a, struct hittable_t *b, int index)
{
	aabb_t bbox_a = a->bounding_box(a);
	aabb_t bbox_b = b->bounding_box(b);
	return aabb_axis(bbox_a, index).min < aabb_axis(bbox_b, index).min;
}

static int
box_x_compare(struct hittable_t *a, struct hittable_t *b)
{
	return box_compare(a, b, 0);
}

static int
box_y_compare(struct hittable_t *a, struct hittable_t *b)
{
	return box_compare(a, b, 1);
}

static int
box_z_compare(struct hittable_t *a, struct hittable_t *b)
{
	return box_compare(a, b, 2);
}

static hittable_t *
sphere_create(point_t center1, float radius, material_t *mat)
{
	hittable_t *hittable;
	hittable = malloc(sizeof(*hittable));
	if (!hittable) {
		fprintf(stderr, "failed to allocate sphere!\n");
		exit(EXIT_FAILURE);
	}

	hittable->type = OBJECT_SPHERE;
	hittable->object.sphere = (sphere_t) { center1, radius, mat, false,
					       center1, SPHERE_AABB(center1, radius)};
	hittable->hit = sphere_hit;
	hittable->bounding_box = sphere_bounding_box;
	return hittable;
}

static hittable_t *
sphere_move_create(point_t center1, point_t center2, float radius, material_t *mat)
{
	hittable_t *hittable;
	hittable = malloc(sizeof(*hittable));
	if (!hittable) {
		fprintf(stderr, "failed to allocate sphere!\n");
		exit(EXIT_FAILURE);
	}

	hittable->type = OBJECT_SPHERE;
	hittable->object.sphere = (sphere_t) { center1, radius, mat, true,
					       SUB(center2, center1),
					       SPHERE_MOVE_AABB(center1, center2, radius)};
	hittable->hit = sphere_hit;
	hittable->bounding_box = sphere_bounding_box;
	return hittable;
}

static bool
sphere_hit(ray_t ray, hittable_t *sphere, interval_t ray_t, hit_record_t *rec)
{
	assert(sphere->type == OBJECT_SPHERE);
	sphere_t object_sphere = sphere->object.sphere;
	point_t center = object_sphere.is_moving ? sphere_center(object_sphere, ray.time)
		: object_sphere.center1;
	vec3_t oc = SUB(ray.origin, center);
	float a = LENGTH_SQUARED(ray.direction);
	float half_b = DOT(oc, ray.direction);
	float c = LENGTH_SQUARED(oc) - object_sphere.radius*object_sphere.radius;
	
	float discriminant = half_b*half_b - a*c;
	if (discriminant < 0) return false;
	float sqrtd = sqrt(discriminant);

	float root = (-half_b - sqrtd) / a;
	if (!interval_surrounds(ray_t, root)) {
		root = (-half_b + sqrtd) / a;
		if (!interval_surrounds(ray_t, root))
			return false;
	}

	rec->t = root;
	rec->p = RAY_AT(ray, rec->t);
	vec3_t outward_normal = ll_vec3_div1f(SUB(rec->p, center),
					      object_sphere.radius);
	set_face_normal(ray, rec, &outward_normal);
	get_sphere_uv(outward_normal, &rec->u, &rec->v);
	rec->mat = object_sphere.mat;
	return true;
}

static aabb_t
sphere_bounding_box(struct hittable_t *hittable)
{
	assert(hittable->type == OBJECT_SPHERE);
	return hittable->object.sphere.bbox;
}

static void
camera_render(struct hit_list_t *list)
{
	int i, j, sample;
	char *progress_bar;

	ray_t ray;
	
	camera_initialize();
	
	progress_bar = malloc(camera.image_height);
	memset(progress_bar, '=', camera.image_height);

	ppm_out = fopen(PPM_PATH, "w");
	if (!ppm_out) {
		fprintf(stderr, "Failed to open '%s'\n", PPM_PATH);
		exit(EXIT_FAILURE);
	}
	
	fprintf(ppm_out, "P3\n%d %d\n255\n", camera.image_width, camera.image_height);
	for (j = 0; j < camera.image_height; j++) {
		printf("\r%f: %.*s", ((j + 1) / (float) camera.image_height) * 100.0,
		       (int) (((j + 1) / (float) camera.image_height) * 40),
		       progress_bar);
		fflush(stdout);
		for (i = 0; i < camera.image_width; i++) {
			colour_t pixel_colour = COLOUR(0, 0, 0);
			for (sample = 0; sample < camera.samples_per_pixel; ++sample) {
				ray_t r = camera_get_ray(i, j);
				pixel_colour = ADD(pixel_colour, camera_ray_colour(&world, r, camera.max_depth));
			}
			write_colour(pixel_colour, camera.samples_per_pixel);
		}
	}
	printf("\nDone.\n");
	free(progress_bar);
}

static void
camera_initialize(void)
{
	camera.image_height = (int) (camera.image_width / camera.aspect_ratio);
	camera.image_height = (camera.image_height < 1) ? 1 : camera.image_height;
	

	camera.center = camera.lookfrom;
	float theta = degrees_to_radians(camera.vfov);
	float h = tan(theta/2);
	float viewport_height = 2 * h * camera.focus_dist;
	float viewport_width = viewport_height
		* (float) (camera.image_width / (float) camera.image_height);

	camera.w = ll_vec3_normalise3fv(SUB(camera.lookfrom, camera.lookat));
	camera.u = ll_vec3_normalise3fv(CROSS(camera.vup, camera.w));
	camera.v = CROSS(camera.w, camera.u);

	vec3_t viewport_u = ll_vec3_mul1f(camera.u, viewport_width);
	vec3_t viewport_v = ll_vec3_mul1f(camera.v, -viewport_height);

	camera.pixel_delta_u = ll_vec3_div1f(viewport_u, camera.image_width);
	camera.pixel_delta_v = ll_vec3_div1f(viewport_v, camera.image_height);

	vec3_t viewport_upper_left = SUB(SUB(SUB(camera.center, ll_vec3_mul1f(camera.w, camera.focus_dist)),
					     ll_vec3_div1f(viewport_u, 2)), ll_vec3_div1f(viewport_v, 2));
	camera.pixel00_loc = ADD(viewport_upper_left,
				 ll_vec3_div1f(ADD(camera.pixel_delta_u,
						   camera.pixel_delta_v), 0.5));
	float defocus_radius = camera.focus_dist * tan(degrees_to_radians(camera.defocus_angle / 2));
	camera.defocus_disk_u = ll_vec3_mul1f(camera.u, defocus_radius);
	camera.defocus_disk_v = ll_vec3_mul1f(camera.v, defocus_radius);
}

static colour_t
camera_ray_colour(hit_list_t *list, ray_t ray, int depth)
{
	if (depth <= 0)
		return COLOUR(0, 0, 0);
	
	if (hit_list_hit(list, ray, INTERVAL2f(0.001, INFINITY))) {
		ray_t scattered;
		colour_t attenuation;
		if (hit_record.mat->scatter(hit_record.mat, ray, &hit_record, &attenuation, &scattered)) {
			return MUL(attenuation, camera_ray_colour(list, scattered, depth - 1));
		}
		return COLOUR(0, 0, 0);
	}
	vec3_t unit_direction = ll_vec3_normalise3fv(ray.direction);
	float a = 0.5 * (unit_direction.y + 1.0);
	return ADD(ll_vec3_mul1f(COLOUR(1.0, 1.0, 1.0), (1.0-a)),
		   ll_vec3_mul1f(COLOUR(0.5, 0.7, 1.0), a));
}

static ray_t
camera_get_ray(int i, int j)
{
	vec3_t iu = ll_vec3_mul1f(camera.pixel_delta_u, i);
	vec3_t jv = ll_vec3_mul1f(camera.pixel_delta_v, j);
	point_t pixel_center = ADD(camera.pixel00_loc, ADD(iu, jv));
	point_t pixel_sample = ADD(pixel_center, camera_pixel_sample_square());

	point_t ray_origin = (camera.defocus_angle <= 0) ? camera.center : defocus_disk_sample();
	vec3_t ray_direction = SUB(pixel_sample, ray_origin);
	float ray_time = random_float();

	return RAY1f(ray_origin, ray_direction, ray_time);
}

static vec3_t
camera_pixel_sample_square(void)
{
	float px = -0.5 + random_float();
	float py = -0.5 + random_float();
	return ADD(ll_vec3_mul1f(camera.pixel_delta_u, px),
		   ll_vec3_mul1f(camera.pixel_delta_v, py));
}

static material_t *
lambertian_create(texture_t *texture)
{
	material_t *material;
	material = malloc(sizeof(*material));
	if (!material) {
		fprintf(stderr, "Failed to create texture!\n");
		exit(EXIT_FAILURE);
	}

	material->type = MATERIAL_LAMBERTIAN;
	material->surface.lambertian = (lambertian_t) { texture };
	material->scatter = lambertian_scatter;
	return material;
}

static bool
lambertian_scatter(struct material_t *material, ray_t ray, struct hit_record_t *hit_record,
		   colour_t *attenuation, ray_t *scattered)
{
	assert(material->type == MATERIAL_LAMBERTIAN);
	lambertian_t lambertian = material->surface.lambertian;
	vec3_t scatter_direction = ADD(hit_record->normal, random_unit_vector());
	if (near_zero(scatter_direction))
		scatter_direction = hit_record->normal;
	*scattered = RAY1f(hit_record->p, scatter_direction, ray.time);
	*attenuation = lambertian.albedo->value(lambertian.albedo, hit_record->u,
						hit_record->v, hit_record->p);
	return true;
}

static bool
metal_scatter(struct material_t *material, ray_t ray, struct hit_record_t *hit_record,
		   colour_t *attenuation, ray_t *scattered)
{
	assert(material->type == MATERIAL_METAL);
	metal_t metal = material->surface.metal;
	vec3_t reflected = reflect(ll_vec3_normalise3fv(ray.direction), hit_record->normal);
	*scattered = RAY1f(hit_record->p, ADD(reflected, ll_vec3_mul1f(random_unit_vector(), metal.fuzz)),
			   ray.time);
	*attenuation = metal.albedo;
	return true;
}

static bool
dielectric_scatter(struct material_t *material, ray_t ray, struct hit_record_t *hit_record,
		   colour_t *attenuation, ray_t *scattered)
{
	assert(material->type == MATERIAL_DIELECTRIC);
	dielectric_t dielectric = material->surface.dielectric;
	*attenuation = COLOUR(1, 1, 1);
	float refraction_ratio = hit_record->front_face ? (1.0/dielectric.ir) : dielectric.ir;

	vec3_t unit_direction = ll_vec3_normalise3fv(ray.direction);
	float cos_theta = fmin(DOT(ll_vec3_mul1f(unit_direction, -1), hit_record->normal), 1.0);
	float sin_theta = sqrt(1.0 - cos_theta*cos_theta);

	bool cannot_refract = refraction_ratio * sin_theta > 1.0;
	vec3_t direction;

	if (cannot_refract || reflectance(cos_theta, refraction_ratio) > random_float()) {
		direction = reflect(unit_direction, hit_record->normal);
	} else {
		direction = refract(unit_direction, hit_record->normal, refraction_ratio);
	}

	*scattered = RAY1f(hit_record->p, direction, ray.time);
	return true;
}

static float
reflectance(float cosine, float ref_idx)
{
	float r0 = (1 - ref_idx) / (1 + ref_idx);
	r0 = r0 * r0;
	return r0 + (1 - r0) * pow((1 - cosine), 5);
}

static point_t
defocus_disk_sample(void)
{
	vec3_t p = random_in_unit_disk();
	return ADD(camera.center, ADD(ll_vec3_mul1f(camera.defocus_disk_u, p.x), ll_vec3_mul1f(camera.defocus_disk_v, p.y)));
}

static aabb_t
aabb_empty(void)
{
	return (aabb_t) { empty, empty, empty };
}

static aabb_t
aabb2v(point_t a, point_t b)
{
	return (aabb_t) {
		.x = INTERVAL2f(fmin(a.x, b.x), fmax(a.x, b.x)),
		.y = INTERVAL2f(fmin(a.y, b.y), fmax(a.y, b.y)),
		.z = INTERVAL2f(fmin(a.z, b.z), fmax(a.z, b.z)),
	};
}

static aabb_t
aabb3i(interval_t x, interval_t y, interval_t z)
{
	return (aabb_t) { x, y, z };
}

static aabb_t
aabb2bbox(aabb_t a, aabb_t b)
{
	return (aabb_t) {
		INTERVAL2v(a.x, b.x),
		INTERVAL2v(a.y, b.y),
		INTERVAL2v(a.z, b.z)
	};
}

static interval_t
aabb_axis(aabb_t aabb, int n)
{
	if (n == 1) return aabb.y;
	if (n == 2) return aabb.z;
	return aabb.x;
}

static bool
aabb_hit(aabb_t aabb, ray_t ray, interval_t ray_t)
{
	for (int a = 0; a < 3; a++) {
		float invD = 1 / ray.direction.data[a];
		float orig = ray.origin.data[a];

		float t0 = (aabb_axis(aabb, a).min - orig) * invD;
		float t1 = (aabb_axis(aabb, a).max - orig) * invD;

		if (invD < 0) {
			float tmp = t0;
			t0 = t1;
			t1 = tmp;
		}

		if (t0 > ray_t.min) ray_t.min = t0;
		if (t1 < ray_t.max) ray_t.max = t1;

		if (ray_t.max <= ray_t.min)
			return false;
	}
	return true;
}

static texture_t *
solid_colour_create3f(float r, float g, float b)
{
	texture_t *texture;
	texture = malloc(sizeof(*texture));
	if (!texture) {
		fprintf(stderr, "Failed to allocate texture!\n");
		exit(EXIT_FAILURE);
	}

	texture->type = TEXTURE_SOLID_COLOUR;
	texture->surface.solid_colour = (solid_colour_t) { COLOUR(r, g, b) };
	texture->value = solid_colour_value;
	return texture;
}

static colour_t
solid_colour_value(texture_t *texture, float u, float v, point_t p)
{
	assert(texture->type == TEXTURE_SOLID_COLOUR);
	return texture->surface.solid_colour.colour;
}

static texture_t *
checkered_create(float scale, texture_t *even, texture_t *odd)
{
	texture_t *texture;
	texture = malloc(sizeof(*texture));
	if (!texture) {
		fprintf(stderr, "Failed to allocate texture!\n");
		exit(EXIT_FAILURE);
	}

	texture->type = TEXTURE_CHECKERED;
	texture->surface.checkered = (checkered_t) { scale, even, odd };
	texture->value = checkered_value;
	return texture;
}

static texture_t *
checkered_create2v(float scale, colour_t c1, colour_t c2)
{
	return checkered_create(scale, solid_colour_create3f(c1.r, c1.g, c1.b),
				solid_colour_create3f(c2.r, c2.g, c2.b));
}

static colour_t
checkered_value(texture_t *texture, float u, float v, point_t p)
{
	assert(texture->type == TEXTURE_CHECKERED);
	int xInteger = (int) (floor(texture->surface.checkered.inv_scale * p.x));
	int yInteger = (int) (floor(texture->surface.checkered.inv_scale * p.y));
	int zInteger = (int) (floor(texture->surface.checkered.inv_scale * p.z));

	bool isEven = (xInteger + yInteger + zInteger) % 2 == 0;
	if (isEven) {
		return texture->surface.checkered.even->value(texture->surface.checkered.even, u, v, p);
	}
	
	return texture->surface.checkered.odd->value(texture->surface.checkered.odd, u, v, p);
}

static texture_t *
image_create(const char *pathname)
{
	texture_t *texture;
	SDL_Surface *img_surface;
	texture = malloc(sizeof(*texture));
	if (!texture) {
		fprintf(stderr, "Failed to create image texture!\n");
		exit(EXIT_FAILURE);
	}
	
	img_surface = IMG_Load(pathname);
	if (!img_surface) {
		fprintf(stderr, "Failed to load '%s'\n", pathname);
		exit(EXIT_FAILURE);
	}

	texture->type = TEXTURE_IMAGE;
	texture->surface.image = (image_t) { img_surface };
	texture->value = image_value;
	return texture;
}

static colour_t
image_value(texture_t *texture, float u, float v, point_t p)
{
	assert(texture->type == TEXTURE_IMAGE);
	if (texture->surface.image.surface->h == 0) {
		return COLOUR(0, 1, 1);
	}

	u = interval_clamp(INTERVAL2f(0, 1), u);
	v = 1.0 - interval_clamp(INTERVAL2f(0, 1), v);

	int i = (int) (u * texture->surface.image.surface->w);
	int j = (int) (v * texture->surface.image.surface->h);

	SDL_PixelFormat *format = texture->surface.image.surface->format;
	void *pixels = texture->surface.image.surface->pixels;
	int BytesPerPixel = format->BytesPerPixel;
	int pitch = texture->surface.image.surface->pitch;

	Uint8 *pixel = (Uint8 *) pixels + j * pitch + i * BytesPerPixel;
	Uint32 pixel_data = *(Uint32 *) pixel;

	float colour_scale = 1.0 / 255.0;
	colour_t colour;

	Uint8 r, g, b;

	SDL_GetRGB(pixel_data, format, &r, &g, &b);
	
	colour = COLOUR(r * colour_scale, g * colour_scale, b * colour_scale);
	return colour;
}

static float
trilinear_interp(float c[2][2][2], float u, float v, float w)
{
	float accum = 0.0;
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			for (int k = 0; k < 2; k++) {
				accum += (i*u + (1-i)*(1-u))*
					(j*v + (1-j)*(1-v))*
					(k*w + (1-k)*(1-w))*c[i][j][k];
			}
		}
	}
	return accum;
}

static float
perlin_interp(vec3_t c[2][2][2], float u, float v, float w)
{
	float uu = u*u*(3-2*u);
	float vv = v*v*(3-2*v);
	float ww = w*w*(3-2*w);
	float accum = 0.0;
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			for (int k = 0; k < 2; k++) {
				vec3_t weight_v = VEC3(u-i, v-j, w-k);
				accum += (i*uu + (1-i)*(1-u))*
					(j*vv + (1-j)*(1-v))*
					(k*ww + (1-k)*(1-w))*
					DOT(c[i][j][k], weight_v);
			}
		}
	}
	return accum;
}

static float
noise(texture_t *texture, point_t p)
{
	assert(texture->type == TEXTURE_NOISE);
	
	float u = p.x - floor(p.x);
	float v = p.y - floor(p.y);
	float w = p.z - floor(p.z);
	

	int i = (int) (floor(p.x));
	int j = (int) (floor(p.y));
	int k = (int) (floor(p.z));

	vec3_t c[2][2][2];
	noise_t noise = texture->surface.noise;

	for (int di = 0; di < 2; di++) {
		for (int dj = 0; dj < 2; dj++) {
			for (int dk = 0; dk < 2; dk++) {
				c[di][dj][dk] = noise.ran_vec[noise.perm_x[(i+di) & 255]
							       ^ noise.perm_y[(j+dj) & 255]
							       ^ noise.perm_z[(k+dk) & 255]];
			}
		}
	}

	return perlin_interp(c, u, v, w);
}

static float
turb(texture_t *texture, point_t p, int depth)
{
	if (depth <= 0) {
		depth = 7;
	}
	
	float accum = 0.0;
	point_t temp_p = p;
	float weight = 1.0;

	for (int i = 0; i < depth; i++) {
		accum += weight*noise(texture, p);
		weight *= 0.5;
		temp_p = ll_vec3_mul1f(temp_p, 2);
	}

	return fabs(accum);
}

static void
permute(int *p, int n)
{
	int i;
	for (i = n - 1; i > 0; i--) {
		int target = random_int2i(0 ,i);
		int tmp = p[i];
		p[i] = p[target];
		p[target] = tmp;
	}
}

static int *
perlin_generate_perm(int point_count)
{
	int i;
	int *p = calloc(point_count, sizeof(*p));
	if (!p) {
		fprintf(stderr, "Failed to allocate points!\n");
		exit(EXIT_FAILURE);
	}
	for (i = 0; i < point_count; i++) {
		p[i] = i;
	}

	permute(p, point_count);
	return p;
}

static texture_t *
noise_create(float sc)
{
	int i;
	texture_t *texture;
	texture = malloc(sizeof(*texture));
	if (!texture) {
		fprintf(stderr, "Failed to create image texture!\n");
		exit(EXIT_FAILURE);
	}

	int point_count = 256;
	vec3_t *ran_vec = malloc(sizeof(*ran_vec) * point_count);
	if (!ran_vec) {
		fprintf(stderr, "Failed to allocate points!\n");
		exit(EXIT_FAILURE);
	}
	for (i = 0; i < point_count; i++) {
		ran_vec[i] = ll_vec3_normalise3f(random_float(), random_float(),
						  random_float());
	}
	
	int *perm_x = perlin_generate_perm(point_count);
	int *perm_y = perlin_generate_perm(point_count);
	int *perm_z = perlin_generate_perm(point_count);

	texture->type = TEXTURE_NOISE;
	texture->surface.noise = (noise_t) { point_count, ran_vec, perm_x, perm_y, perm_z, sc };
	texture->value = noise_value;
	return texture;
}

static colour_t
noise_value(texture_t *texture, float u, float v, point_t p)
{
	assert(texture->type == TEXTURE_NOISE);

	p = ll_vec3_mul1f(p, texture->surface.noise.scale);
	
	return ll_vec3_mul1f(COLOUR(0.5, 0.5, 0.5), (1.0 + sin(p.z + 10.0*turb(texture, p, 7))));
}





























