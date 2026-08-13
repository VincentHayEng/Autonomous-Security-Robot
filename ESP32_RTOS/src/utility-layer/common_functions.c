#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

uint8_t * allocate_memory(uint16_t size)
{
	return calloc(size, sizeof(uint8_t));
}
