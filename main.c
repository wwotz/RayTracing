#define LINEARLIB_IMPLEMENTATION
#include "linear.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <assert.h>

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
random_on_hemisphere(vec3_t normal)
{
	vec3_t on_unit_sphere = random_unit_vector();
	if (DOT(on_unit_sphere, normal) > 0.0)
		return on_unit_sphere;
	return ll_vec3_mul1f(on_unit_sphere, -1.0);
}

static inline float
linear_to_gamma(float linear_component)
{
	return sqrt(linear_component);
}


typedef struct interval_t {
	float min, max;
} interval_t;

static bool
interval_contains(interval_t interval, float x);

static bool
interval_surrounds(interval_t interval, float x);

static float
interval_clamp(interval_t interval, float x);

#define INTERVAL(a, b) (interval_t) { a, b }

const static interval_t empty    = INTERVAL(+INFINITY, -INFINITY);
const static interval_t universe = INTERVAL(-INFINITY, +INFINITY);

typedef struct ray_t {
	point_t origin;
	vec3_t direction;
} ray_t;

#define RAY(origin, direction) (ray_t) { origin, direction }
#define RAY_AT(ray, t) ADD(ray.origin, ll_vec3_mul1f(ray.direction, t))

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
} camera;

static void
camera_render(void);

static void
camera_initialize(void);

static colour_t
camera_ray_colour(ray_t ray, int depth);

static ray_t
camera_get_ray(int i, int j);

static vec3_t
camera_pixel_sample_square(void);

typedef struct hit_record_t {
	point_t p;
	vec3_t normal;
	float t;
	bool front_face;
} hit_record_t;

typedef enum obj_t {
	OBJECT_SPHERE,
	OBJECT_COUNT,
} obj_t;

typedef struct sphere_t {
	point_t center;
	float radius;
} sphere_t;

typedef struct hittable_t {
	obj_t type;
	union {
		struct sphere_t sphere;
	} object;
	bool (*hit)(ray_t ray, struct hittable_t hittable, interval_t interval,
		    hit_record_t *hit_record);
} hittable_t;

static hit_record_t hit_record;

static bool
sphere_hit(ray_t ray, hittable_t sphere, interval_t ray_t, hit_record_t *hit_record);

#define SPHERE(center, radius) (hittable_t) { OBJECT_SPHERE,		\
			.object.sphere = (sphere_t) { center, radius }, .hit = sphere_hit } \
	
struct {
	size_t size;
	size_t capacity;
	hittable_t *list;
} hit_list;

#define HIT_LIST_DEFAULT_CAPACITY 8

static void
hit_list_push_back(hittable_t hittable);
static bool
hit_list_hit(ray_t ray, interval_t ray_t);

static void
set_face_normal(ray_t ray, hit_record_t *rec, const vec3_t *outward_normal);

static void
write_colour(colour_t colour, int samples_per_pixel);

static float
hit_sphere(point_t center, float radius);

int
main(int argc, char **argv)
{
	hit_list_push_back(SPHERE(POINT(0, 0, -1), 0.5));
	hit_list_push_back(SPHERE(POINT(0, -100.5, -1), 100));

	camera.image_width = IMAGE_WIDTH;
	camera.aspect_ratio = ASPECT_RATIO;
	camera.samples_per_pixel = 100;
	camera.max_depth = 50;

	camera_render();
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

	static const interval_t intensity = INTERVAL(0.000, 0.999);
	fprintf(ppm_out, "%d %d %d\n",
		(int) (256 * interval_clamp(intensity, r)),
		(int) (256 * interval_clamp(intensity, g)),
		(int) (256 * interval_clamp(intensity, b)));
}

static void
hit_list_push_back(hittable_t hittable)
{
	void *new_list;
	if (hit_list.list) {
		if (hit_list.size == hit_list.capacity) {
			new_list = realloc(hit_list.list, hit_list.capacity*2);
			if (!new_list) {
				fprintf(stderr, "Failed to resize list!\n");
				exit(EXIT_FAILURE);
			}
			hit_list.list = new_list;
			hit_list.capacity *= 2;
		}
	} else {
		hit_list.capacity = HIT_LIST_DEFAULT_CAPACITY;
		hit_list.list = calloc(hit_list.capacity, sizeof(*hit_list.list));
		if (!hit_list.list) {
			fprintf(stderr, "Failed to allocate list!\n");
			exit(EXIT_FAILURE);
		}
	}

	hit_list.list[hit_list.size++] = hittable;
}

static bool
hit_list_hit(ray_t ray, interval_t ray_t)
{
	size_t i;
	hit_record_t temp_rec;
	bool hit_anything = false;
	float closest_so_far = ray_t.max;
	hittable_t hittable;

	for (i = 0; i < hit_list.size; i++) {
		hittable = hit_list.list[i];
		if (hittable.hit(ray, hittable, INTERVAL(ray_t.min, closest_so_far), &temp_rec)) {
			hit_anything = true;
			closest_so_far = temp_rec.t;
			hit_record = temp_rec;
		}
	}
	
	return hit_anything;
}

static void
set_face_normal(ray_t ray, hit_record_t *rec, const vec3_t *outward_normal)
{
	rec->front_face = DOT(ray.direction, *outward_normal) < 0;
	rec->normal = rec->front_face ? *outward_normal
		: ll_vec3_mul1f(*outward_normal, -1.0);
}

static bool
sphere_hit(ray_t ray, hittable_t sphere, interval_t ray_t, hit_record_t *rec)
{
	assert(sphere.type == OBJECT_SPHERE);
	sphere_t object_sphere = sphere.object.sphere;
	vec3_t oc = SUB(ray.origin, object_sphere.center);
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
	vec3_t outward_normal = ll_vec3_div1f(SUB(rec->p, object_sphere.center),
					      object_sphere.radius);
	set_face_normal(ray, rec, &outward_normal);
	return true;
}

static void
camera_render(void)
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
		for (i = 0; i < camera.image_width; i++) {
			colour_t pixel_colour = COLOUR(0, 0, 0);
			for (sample = 0; sample < camera.samples_per_pixel; ++sample) {
				ray_t r = camera_get_ray(i, j);
				pixel_colour = ADD(pixel_colour, camera_ray_colour(r, camera.max_depth));
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
	

	camera.center = POINT(0, 0, 0);
	float focal_length = 1.0;
	float viewport_height = 2.0;
	float viewport_width = viewport_height
		* (float) (camera.image_width / (float) camera.image_height);

	vec3_t viewport_u = VEC3(viewport_width, 0, 0);
	vec3_t viewport_v = VEC3(0, -viewport_height, 0);

	camera.pixel_delta_u = ll_vec3_div1f(viewport_u, camera.image_width);
	camera.pixel_delta_v = ll_vec3_div1f(viewport_v, camera.image_height);

	vec3_t viewport_upper_left = SUB(SUB(SUB(camera.center, VEC3(0, 0, focal_length)),
					     ll_vec3_div1f(viewport_u, 2)), ll_vec3_div1f(viewport_v, 2));
	camera.pixel00_loc = ADD(viewport_upper_left,
				 ll_vec3_div1f(ADD(camera.pixel_delta_u,
						   camera.pixel_delta_v), 0.5));
}

static colour_t
camera_ray_colour(ray_t ray, int depth)
{
	if (depth <= 0)
		return COLOUR(0, 0, 0);
	
	if (hit_list_hit(ray, INTERVAL(0.001, INFINITY))) {
		vec3_t direction = ADD(hit_record.normal, random_unit_vector());
		//return ll_vec3_mul1f(ADD(hit_record.normal, COLOUR(1, 1, 1)), 0.5);
		return ll_vec3_mul1f(camera_ray_colour(RAY(hit_record.p, direction), depth - 1), 0.5);
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

	point_t ray_origin = camera.center;
	vec3_t ray_direction = SUB(pixel_sample, ray_origin);

	return RAY(ray_origin, ray_direction);
}

static vec3_t
camera_pixel_sample_square(void)
{
	float px = -0.5 + random_float();
	float py = -0.5 + random_float();
	return ADD(ll_vec3_mul1f(camera.pixel_delta_u, px),
		   ll_vec3_mul1f(camera.pixel_delta_v, py));
}



































