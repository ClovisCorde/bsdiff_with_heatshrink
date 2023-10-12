/*-
 * Copyright 2003-2005 Colin Percival
 * Copyright 2012 Matthew Endsley
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted providing that the following conditions 
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <limits.h>
#include <bzlib.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <err.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include "bspatch.h"


#define INPUT_READ_SIZE HEATSHRINK_STATIC_INPUT_BUFFER_SIZE
#define OUTPUT_BUFFER_SIZE 2048
#define OUTPUT_READ_SIZE 256
#define HEADER_SIZE 8

static int decode(FILE *input, uint8_t *output_buffer, size_t output_buffer_size, heatshrink_decoder *hsd, size_t *bytes_written, decoder_state_t *state) {
    uint8_t input_buffer[INPUT_READ_SIZE];
    size_t read_sz = 0;
    size_t write_sz = 0;
    size_t offset_buffer = 0;
    static size_t offset_file = 16;

    /* Get size of the compressed file */
    fseek(input, 0L, SEEK_END);
    size_t file_size = ftell(input);
    fseek(input, offset_file, SEEK_SET);

    size_t bytes_to_decode = output_buffer_size;
    *state = STATE_DECODE;

    while (offset_file <= file_size)
    {
        switch (*state)
        {
        case STATE_DECODE: ;
            size_t input_read = fread(input_buffer, 1, 1, input);
            offset_file += input_read;
     
            if(input_read == 0) {
                *state = STATE_FINISH;
                break;
            }
            HSD_sink_res sink_res = heatshrink_decoder_sink(hsd, input_buffer, input_read, &read_sz);
            if (sink_res != 0) {
                if(sink_res == 1) {
                    printf("Out of space in internal buffer\n");
                    return sink_res;
                } else if(sink_res == -1) {
                    printf("NULL Argument\n");
                    return sink_res;
                } else {
                    printf("Unknown error\n");
                    return sink_res;
                }
            }
            *state = STATE_POLL;
            break;
        case STATE_POLL: ;
            HSD_poll_res poll_res = HSDR_POLL_MORE;
            while (poll_res != HSDR_POLL_EMPTY && bytes_to_decode > 0) {
                size_t space_left = output_buffer_size - offset_buffer;
                size_t to_decode = space_left < OUTPUT_READ_SIZE ? space_left : OUTPUT_READ_SIZE;
                poll_res = heatshrink_decoder_poll(hsd, output_buffer + offset_buffer, to_decode, &write_sz);
                offset_buffer += write_sz;
                bytes_to_decode -= write_sz;
            }
            if(bytes_to_decode == 0) {
                *state = STATE_FINISH;
            }
            if(poll_res == HSDR_POLL_EMPTY) {
                *state = STATE_DECODE;
            }
            
            break;
        case STATE_FINISH: ;
            HSD_finish_res finish_res = heatshrink_decoder_finish(hsd);
            if (offset_buffer > 0) {
                *bytes_written = offset_buffer;
            }
            
            return finish_res;
            break;
        
        default:
            break;
        }
    }
    return -1;
}

static int64_t offtin(uint8_t *buf)
{
	int64_t y;

	y=buf[7]&0x7F;
	y=y*256;y+=buf[6];
	y=y*256;y+=buf[5];
	y=y*256;y+=buf[4];
	y=y*256;y+=buf[3];
	y=y*256;y+=buf[2];
	y=y*256;y+=buf[1];
	y=y*256;y+=buf[0];

	if(buf[7]&0x80) y=-y;

	return y;
}

int bspatch(size_t newsize, struct bspatch_stream* stream, char *old_path, char *new_path, FILE *patch_file, uint8_t *output_buffer, size_t *offset_buffer, heatshrink_decoder *hsd, size_t *bytes_written, decoder_state_t *state)
{
	uint8_t buf[HEADER_SIZE];
	size_t oldpos = 0;
	size_t newpos = 0;
	int64_t ctrl[3];
	uint8_t old[OUTPUT_READ_SIZE];
	uint8_t new[OUTPUT_READ_SIZE];

	FILE* fpold = fopen(old_path, "rb");
	FILE* fpnew = fopen(new_path, "wb");

	while(newpos<newsize) {
        if(*bytes_written == 0) {
            decode(patch_file, output_buffer, 24, hsd, bytes_written, state);
            heatshrink_decoder_reset(hsd);
            *offset_buffer = 0;
        }
		/* Read control data */
		for(size_t i=0; i<=2; i++) {
            memcpy(buf, output_buffer + *offset_buffer, HEADER_SIZE);
            *offset_buffer += HEADER_SIZE;
            *bytes_written -= HEADER_SIZE;
			ctrl[i]=offtin(buf);
		};
        printf("ctrl[0] = %ld, ctrl[1] = %ld, ctrl[2] = %ld\n", ctrl[0], ctrl[1], ctrl[2]);
        
		/* Sanity-check */
		if (ctrl[0]<0 || ctrl[0]>INT_MAX ||
			ctrl[1]<0 || ctrl[1]>INT_MAX ||
			newpos+ctrl[0]>newsize)
			return -1;

		while(ctrl[0] > 0) {
            if(*bytes_written == 0) {
                int64_t decoded = ctrl[0] > OUTPUT_BUFFER_SIZE ? OUTPUT_BUFFER_SIZE : ctrl[0];
                decode(patch_file, output_buffer, decoded, hsd, bytes_written, state);
                if(*bytes_written < OUTPUT_BUFFER_SIZE) {
                    heatshrink_decoder_reset(hsd);
                }
                *offset_buffer = 0;
            }
			int64_t bytesToRead = ctrl[0] > OUTPUT_READ_SIZE ? OUTPUT_READ_SIZE : ctrl[0];
            if(bytesToRead > *bytes_written) {
                bytesToRead = *bytes_written;
            }
            
            memcpy(new, output_buffer + *offset_buffer, bytesToRead);
            *offset_buffer += bytesToRead;
            *bytes_written -= bytesToRead;

			/* Add old data to diff string */
			fseek(fpold, oldpos, SEEK_SET);
			fread(old, sizeof(uint8_t), bytesToRead, fpold);
			for(size_t i = 0; i < bytesToRead; i++) {
				new[i] += old[i];
			}

			/* Write the diff string to the new file */
			fwrite(new, sizeof(uint8_t), bytesToRead, fpnew);

			/* Adjust pointers */
			newpos += bytesToRead;
			oldpos += bytesToRead;
			ctrl[0] -= bytesToRead;
		}

		/* Sanity-check */
		if(newpos+ctrl[1]>newsize)
			return -1;

		/* Process the extra string */
		while(ctrl[1] > 0) {
            if(*bytes_written == 0) {
                int64_t decoded = ctrl[1] > OUTPUT_BUFFER_SIZE ? OUTPUT_BUFFER_SIZE : ctrl[1];
                decode(patch_file, output_buffer, decoded, hsd, bytes_written, state);
                if(*bytes_written < OUTPUT_BUFFER_SIZE) {
                    heatshrink_decoder_reset(hsd);
                }
                *offset_buffer = 0;
            }
			int64_t bytesToRead = ctrl[1] > OUTPUT_READ_SIZE ? OUTPUT_READ_SIZE : ctrl[1];
            if(bytesToRead > *bytes_written) {
                bytesToRead = *bytes_written;
            }

            memcpy(new, output_buffer + *offset_buffer, bytesToRead);
            *offset_buffer += bytesToRead;
            *bytes_written -= bytesToRead;
			/* Write the extra string to the new file */
			fwrite(new, sizeof(uint8_t), bytesToRead, fpnew);

			/* Adjust pointers */
			newpos += bytesToRead;
			ctrl[1] -= bytesToRead;
		}
		oldpos+=ctrl[2];
	};

	return 0;
}

#if defined(BSPATCH_EXECUTABLE)

static int file_read(const struct bspatch_stream* stream, void* buffer, int length)
{
    printf("length = %d\n", length);
	int n;
	FILE* file;

	file = (FILE*)stream->opaque;
	n = fread(buffer, 1, length, file);
	if (n != length)
		return -1;

	return 0;
}

int main(int argc,char * argv[])
{
	FILE * patch_file;
	uint8_t header[HEADER_SIZE];
	size_t newsize;
	struct bspatch_stream stream;

    heatshrink_decoder hsd;
    heatshrink_decoder_reset(&hsd);
    uint8_t output_buffer[OUTPUT_BUFFER_SIZE];
    size_t bytes_written = 0;
    size_t offset_buffer = 0;
    decoder_state_t state = STATE_DECODE;

    if(argc!=4) errx(1,"usage: %s oldfile newfile patchfile\n",argv[0]);

    /* Open patch file */
	if ((patch_file = fopen(argv[3], "r")) == NULL)
		err(1, "fopen(%s)", argv[3]);

    int res = fread(output_buffer, 1, 16, patch_file);
    if(res != 16) {
        err(1, "error reading magic header");
    }

    memcpy(header, output_buffer, HEADER_SIZE);
    if(memcmp(header, "BSDIFFHS", 8) != 0)
        errx(1, "Corrupt patch\n");
    
    memcpy(header, output_buffer + 8, HEADER_SIZE);

	/* Read lengths from header */
	newsize=offtin(header);
    printf("newsize = %ld\n", newsize);
	if(newsize<0)
		errx(1,"Corrupt patch\n");
    
	stream.read = file_read;
	stream.opaque = patch_file;
	if (bspatch(newsize, &stream, argv[1], argv[2], patch_file, output_buffer, &offset_buffer, &hsd, &bytes_written, &state))
		errx(1, "bspatch");

	fclose(patch_file);
	return 0;
}

#endif
