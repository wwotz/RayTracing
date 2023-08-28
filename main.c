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

#define IMAGE_WIDTH  400
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

struct {
	point_t origin;
	vec3_t direction;
} ray;

#define RAY_AT(t) ADD(ray.origin, ll_vec3_mul1f(ray.direction, t))

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
	bool (*hit)(struct hittable_t hittable, float ray_tmin,
		    float ray_tmax, hit_record_t *hit_record);
} hittable_t;

static hit_record_t hit_record;

static bool
sphere_hit(hittable_t sphere, float ray_tmin, float ray_tmax, hit_record_t *hit_record);

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
hit_list_hit(float ray_tmin, float ray_tmax);

static void
set_face_normal(hit_record_t *rec, const vec3_t *outward_normal);

static void
write_colour(colour_t colour);

static colour_t
ray_colour(void);

static float
hit_sphere(point_t center, float radius);

int
main(int argc, char **argv)
{
	int i, j;
	char *progress_bar;
	int image_width = IMAGE_WIDTH;
	float aspect_ratio = ASPECT_RATIO;
	int image_height = (int) (image_width / aspect_ratio);
	image_height = (image_height < 1) ? 1 : image_height;

	hit_list_push_back(SPHERE(POINT(0, 0, -1), 0.5));
	hit_list_push_back(SPHERE(POINT(0, -100.5, -1), 100));

	float focal_length = 1.0;
	float viewport_height = 2.0;
	float viewport_width = viewport_height
		* (float) (image_width / (float) image_height);
	point_t camera_center = POINT(0, 0, 0);

	vec3_t viewport_u = VEC3(viewport_width, 0, 0);
	vec3_t viewport_v = VEC3(0, -viewport_height, 0);

	vec3_t pixel_delta_u = ll_vec3_div1f(viewport_u, image_width);
	vec3_t pixel_delta_v = ll_vec3_div1f(viewport_v, image_height);

	vec3_t viewport_upper_left = SUB(SUB(SUB(camera_center, VEC3(0, 0, focal_length)),
					     ll_vec3_div1f(viewport_u, 2)), ll_vec3_div1f(viewport_v, 2));
	vec3_t pixel00_loc = ADD(viewport_upper_left,
				 ll_vec3_div1f(ADD(pixel_delta_u, pixel_delta_v), 0.5));

	progress_bar = malloc(image_height);
	memset(progress_bar, '=', image_height);

	ppm_out = fopen(PPM_PATH, "w");
	if (!ppm_out) {
		fprintf(stderr, "Failed to open '%s'\n", PPM_PATH);
		exit(EXIT_FAILURE);
	}
	
	fprintf(ppm_out, "P3\n%d %d\n255\n", image_width, image_height);
	for (j = 0; j < image_height; j++) {
		printf("\r%f: %.*s", ((j + 1) / (float) image_height) * 100.0,
		       (int) (((j + 1) / (float) image_height) * 40),
		       progress_bar);
		for (i = 0; i < image_width; i++) {
			point_t pixel_center = ADD(pixel00_loc, ADD(ll_vec3_mul1f(pixel_delta_u, i),
								    ll_vec3_mul1f(pixel_delta_v, j)));
			vec3_t ray_direction = SUB(pixel_center, camera_center);
			
			ray.origin = camera_center;
			ray.direction = ray_direction;
				
			colour_t pixel_colour = ray_colour();
			write_colour(pixel_colour);
		}
	}
	printf("\nDone.\n");
	return 0;
}

static void
write_colour(colour_t colour)
{
	int ir = (int) (255.999 * colour.r);
	int ig = (int) (255.999 * colour.g);
	int ib = (int) (255.999 * colour.b);
	fprintf(ppm_out, "%d %d %d\n", ir, ig, ib);
}

static colour_t
ray_colour(void)
{
	if (hit_list_hit(0, INFINITY)) {
		return ll_vec3_mul1f(ADD(hit_record.normal, COLOUR(1, 1, 1)), 0.5);
	}
	vec3_t unit_direction = ll_vec3_normalise3fv(ray.direction);
	float a = 0.5 * (unit_direction.y + 1.0);
	return ADD(ll_vec3_mul1f(COLOUR(1.0, 1.0, 1.0), (1.0-a)),
		   ll_vec3_mul1f(COLOUR(0.5, 0.7, 1.0), a));
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
hit_list_hit(float ray_tmin, float ray_tmax)
{
	size_t i;
	hit_record_t temp_rec;
	bool hit_anything = false;
	float closest_so_far = ray_tmax;
	hittable_t hittable;

	for (i = 0; i < hit_list.size; i++) {
		hittable = hit_list.list[i];
		if (hittable.hit(hittable, ray_tmin, closest_so_far, &temp_rec)) {
			hit_anything = true;
			closest_so_far = temp_rec.t;
			hit_record = temp_rec;
		}
	}
	
	return hit_anything;
}

static void
set_face_normal(hit_record_t *rec, const vec3_t *outward_normal)
{
	rec->front_face = DOT(ray.direction, *outward_normal) < 0;
	rec->normal = rec->front_face ? *outward_normal
		: ll_vec3_mul1f(*outward_normal, -1.0);
}

static bool
sphere_hit(hittable_t sphere, float ray_tmin, float ray_tmax, hit_record_t *rec)
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
	if (root <= ray_tmin || ray_tmax <= root) {
		root = (-half_b + sqrtd) / a;
		if (root <= ray_tmin || ray_tmax <= root)
			return false;
	}

	rec->t = root;
	rec->p = RAY_AT(rec->t);
	vec3_t outward_normal = ll_vec3_div1f(SUB(rec->p, object_sphere.center),
					      object_sphere.radius);
	set_face_normal(rec, &outward_normal);
	return true;
}





































