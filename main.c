#define LINEARLIB_IMPLEMENTATION
#include "linear.h"

#include <stdio.h>

#define PPM_LOC "output.ppm"

typedef vec3_t colour_t;
typedef vec3_t point_t;
typedef vec3_t direction_t;

/**
 * @description The file descriptor to write the
 * ppm file to.
 */
FILE *ppm_out;

typedef struct ray_t {
	point_t origin;
	direction_t direction;
} ray_t;

static point_t
ray_at(ray_t ray, float t)
{
	return ll_vec3_add3fv(ray.origin, ll_vec3_mul1f(ray.direction, t));
}

static void
write_colour(colour_t pixel_colour)
{
	fprintf(ppm_out, "%d %d %d\n",
		(int) (255.999 * pixel_colour.r),
		(int) (255.999 * pixel_colour.g),
		(int) (255.999 * pixel_colour.b));
}

int
main(int argc, char **argv)
{
	ppm_out = fopen(PPM_LOC, "w");
	int image_width = 256;
	int image_height = 256;

	fprintf(ppm_out, "P3\n%d %d\n255\n", image_width, image_height);
	for (int j = 0; j < image_height; j++) {
		fprintf(stdout, "\rScanlines remaining: %d", (image_height - j));
		fflush(stdout);
		for (int i = 0; i < image_width; i++) {
			colour_t pixel_colour = ll_vec3_create3f((float) i / (image_width - 1),
								 (float) j / (image_height - 1),
								 0.0);
			write_colour(pixel_colour);
		}
	}

	fprintf(stdout, "\nDone!\n");
	fclose(ppm_out);
	return 0;
}
