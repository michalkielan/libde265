/*
  libde265 example application "sherlock265".

  MIT License

  Copyright (c) 2013-2014 struktur AG, Dirk Farin <farin@struktur.de>

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include <getopt.h>
#include "VideoPlayer.hh"


typedef struct arg_values {
  char input_filename[1024];
  int debug;
  int nrem;
  char** rem;
} arg_values;

static arg_values values;

#define DEFAULT_DEBUG 0
#define DEFAULT_INPUT_FILENAME ""

static struct option longopts[] = {
    // options with no argument
    {"debug", no_argument, &values.debug, values.debug+1},
    {"quiet", no_argument, &values.debug, 0},
    // matching options to short options
    {"input", required_argument, NULL, 'i'},
    {"output", required_argument, NULL, 'o'},
    {NULL, 0, NULL, 0}
};

void usage(char *name)
{
  fprintf(stderr,"usage: sherlock265 [options] videofile.bin\n");
  fprintf(stderr,"where:\n");
  fprintf(stderr,"  -d: increase debug information\n");
  fprintf(stderr,"  -q: quiet (no debug information\n");
  fprintf(stderr,"  --tint-qp: tint QP information\n");
  fprintf(stderr,"videofile.bin: a raw h.265 bitstream (e.g. HM-10.0 output)\n");
}

arg_values *parse_args (int argc, char** argv) {
  strcpy(values.input_filename, DEFAULT_INPUT_FILENAME);
  values.debug = DEFAULT_DEBUG;

  // getopt_long stores the option index here
  int optindex = 0;
  // long options

  static struct option longopts[] = {
      // options with no argument
      {"debug", no_argument, &values.debug, values.debug+1},
      {"quiet", no_argument, &values.debug, 0},
      {NULL, 0, NULL, 0}
  };

  int c;
  while ((c = getopt_long(argc, argv, "dqh?", longopts, &optindex)) != -1)
  {
    switch (c)
    {
      case 0:
        // long options that define flag
        if (longopts[optindex].flag != NULL) {
          break;
        }
        printf ("option %s", longopts[optindex].name);
        if (optarg) {
          printf (" with arg %s", optarg);
        }
        printf ("\n");
        break;

      case 'd':
        values.debug += 1;
        break;

      case 'q':
        values.debug = 0;
        break;

      case 'h':
      case '?':
      default:
        usage(argv[0]);
        exit(0);
        break;

    }
  }

  // parse non-option arguments
  if (argc > optind) {
    strncpy(values.input_filename, argv[optind], 1023);
    ++optind;
  }

  // store remaining arguments
  values.nrem = argc - optind;
  values.rem = argv + optind;

  return &values;
}

int main(int argc, char **argv)
{

  arg_values *values;
  values = parse_args(argc, argv);
  if (values == NULL) {
    usage(argv[0]);
    exit(-1);
  }

  if (values->nrem != 0) {
    fprintf(stderr,"usage: sherlock265 videofile.bin\n");
    fprintf(stderr,"The video file must be a raw h.265 bitstream (e.g. HM-10.0 output)\n");
    exit(5);
  }


  QApplication app(argc, argv);

  VideoPlayer videoPlayer(values->input_filename);
  videoPlayer.show();

  return app.exec();
}
