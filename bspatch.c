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

#define BLOCK_SIZE 256

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

int bspatch(size_t newsize, struct bspatch_stream* stream, char *old_path, char *new_path)
{
	uint8_t buf[8];
	size_t oldpos = 0;
	size_t newpos = 0;
	int64_t ctrl[3];
	uint8_t old[BLOCK_SIZE];
	uint8_t new[BLOCK_SIZE];

	FILE* fpold = fopen(old_path, "rb");
	FILE* fpnew = fopen(new_path, "wb");

	while(newpos<newsize) {
		/* Read control data */
		for(size_t i=0; i<=2; i++) {
			if (stream->read(stream, buf, 8))
				return -1;
			ctrl[i]=offtin(buf);
		};

		/* Sanity-check */
		if (ctrl[0]<0 || ctrl[0]>INT_MAX ||
			ctrl[1]<0 || ctrl[1]>INT_MAX ||
			newpos+ctrl[0]>newsize)
			return -1;

		while(ctrl[0] > 0) {
			int64_t bytesToRead = ctrl[0] > BLOCK_SIZE ? BLOCK_SIZE : ctrl[0];
			if(stream->read(stream, new, bytesToRead)){
				printf("Error reading diff string ctrl[0]\n");
				return -1;
			}

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
			int64_t bytesToRead = ctrl[1] > BLOCK_SIZE ? BLOCK_SIZE : ctrl[1];
			if(stream->read(stream, new, bytesToRead)) {
				printf("Error reading extra string ctrl[1]\n");
				return -1;
			}

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
	uint8_t header[8];
	size_t newsize;
	struct bspatch_stream stream;

	if(argc!=4) errx(1,"usage: %s oldfile newfile patchfile\n",argv[0]);

	/* Open patch file */
	if ((patch_file = fopen(argv[3], "r")) == NULL)
		err(1, "fopen(%s)", argv[3]);

	/* Read header */
	if (fread(header, 1, 8, patch_file) != 8) {
		if (feof(patch_file))
			errx(1, "Corrupt patch\n");
		err(1, "fread(%s)", argv[3]);
	}

	/* Read lengths from header */
	newsize=offtin(header);
	if(newsize<0)
		errx(1,"Corrupt patch\n");


	stream.read = file_read;
	stream.opaque = patch_file;
	if (bspatch(newsize, &stream, argv[1], argv[2]))
		errx(1, "bspatch");

	fclose(patch_file);

	return 0;
}

#endif
