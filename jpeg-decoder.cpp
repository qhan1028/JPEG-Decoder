/*
 *  ITCT JPEG Decoder
 *  Written by Liang-Han, Lin
 *  2019.4.17
 */

#include <fstream>
#include <iostream>
#include <map>
#include <math.h>
#include <stdio.h>
#include <sstream>
#include <string>
#include <string.h>
#include <vector>

#include "bitmap-image.hpp"

#define PI 3.14159265359

#define TAG 0xFF
#define SOI 0xD8
#define DQT 0xDB
#define SOF0 0xC0
#define SOF2 0xC2
#define EOI 0xD9
#define DHT 0xC4
#define SOS 0xDA
#define COM 0xFE
#define DRI 0xDD

#define R 0
#define G 1
#define B 2

using namespace std;


/*
 *
 *  Global Variables
 *
 */
bool debug = false;

bool USE_RST = false;
uint8_t zzi[64] = {  0,  1,  5,  6, 14, 15, 27, 28,
                     2,  4,  7, 13, 16, 26, 29, 42, 
                     3,  8, 12, 17, 25, 30, 41, 43,
                     9, 11, 18, 24, 31, 40, 44, 53,
                    10, 19, 23, 32, 39, 45, 52, 54,
                    20, 22, 33, 38, 46, 51, 55, 60,
                    21, 34, 37, 47, 50, 56, 59, 61,
                    35, 36, 48, 49, 57, 58, 62, 63};


/*
 *
 *  Utilities
 * 
 */
inline unsigned to_uint(char c) {return (unsigned)(unsigned char)c;}

inline unsigned to_num(char c1, char c2) {return to_uint(c1) << 8 | to_uint(c2);}

void skip(ifstream& jpegfile) {
    char c1, c2;
    jpegfile.read(&c1, 1);
    jpegfile.read(&c2, 1);
    
    unsigned int length = to_num(c1, c2);

    if (debug)
        cout << length << endl;

    jpegfile.seekg(length - 2, ios::cur);
}

typedef struct buffer {
    uint8_t valid_bits = 0;
    uint32_t bits = 0;
    const uint32_t msb_mask = 1 << 31;
} buffer_t;

uint16_t read_bits(ifstream& jpegfile, buffer_t& b, vector<uint16_t>& prev_dc, uint8_t n_bits, bool is_symbol) {
    char c;

    if (n_bits > b.valid_bits) {

        do {
            jpegfile.read(&c, 1);
            uint8_t byte = to_uint(c);

            if (byte == TAG) {
                jpegfile.read(&c, 1);
                if (USE_RST) { // reset dc prev
                    switch (to_uint(c)) {
                        case 0xD0: case 0xD1: case 0xD2: case 0xD3:
                        case 0xD4: case 0xD5: case 0xD6: case 0xD7:
                            for (int i = 0; i < prev_dc.size(); i++) prev_dc[i] = 0;
                            break;
                        default:;
                    }
                }
            }

            b.bits = b.bits | (to_uint(byte) << (24 - b.valid_bits));
            b.valid_bits += 8;

            //cout << b.bits << ", n_bits: " << (int)n_bits << ", valid_bits: " << (int)b.valid_bits << endl;

        } while (n_bits > b.valid_bits);
    }

    uint16_t output = 0;

    if (is_symbol) {
        output = ((b.bits >> (31)) & ((1 << 1) - 1));

        b.valid_bits -= 1;
        b.bits = b.bits << 1;

    } else {
        if(((b.bits >> (32 - 1)) & ((1 << 1) - 1)) == 0){
            output = ((b.bits >> (32 - n_bits)) | ((-1) << n_bits));
            output += 1; //why?

        } else {
            output = ((b.bits >> (32 - n_bits)) & ((1 << n_bits) - 1));
        }
        
        b.valid_bits -= n_bits;
        b.bits = b.bits << n_bits;  

        // cout << "msb: " << (int)(b.bits >> (31)) << ", shift: " << (int)((b.bits >> (31)) & ((1 << 1) - 1)) << ", output: " << (int16_t)output << endl;
    }
    
    return output;
}

void fast_idct(int16_t* vec, int rc, bool by_row){
    const int FACTOR = 1;
    double a = PI / 16.0;

    double c1 = double(FACTOR * cos(4.0 * a));
    double c2 = double(FACTOR * sin(4.0 * a));
    double c3 = double(FACTOR * cos(a));
    double c4 = double(FACTOR * sin(a));
    double c5 = double(FACTOR * cos(2.0 * a));
    double c6 = double(FACTOR * sin(2.0 * a));
    double c7 = double(FACTOR * cos(3.0 * a));
    double c8 = double(FACTOR * sin(3.0 * a));
    double c9 = double(FACTOR * sqrt(2.0));

    int si1[8] = {4, 0, 1, 7, 2, 6, 3, 5};
    double s1[8];

    if(by_row) {
        for (int i = 0; i < 8; i++) {
            s1[i] = (double)(vec[rc * 8 + si1[i]] * 8);
        }
    } else {
        for (int i = 0; i < 8; i++) {
            s1[i] = (double)(vec[si1[i] * 8 + rc] * 8);
        }
    }

    double s2[8];
    s2[0] = (c1 * s1[0] + c2 * s1[1]) * 2.0;
    s2[1] = (c5 * s1[4] + c6 * s1[5]) * 2.0;
    s2[2] = (c1 * s1[1] - c2 * s1[0]) * 2.0;
    s2[3] = (c5 * s1[5] - c6 * s1[4]) * 2.0;
    s2[4] = (c3 * s1[2] + c4 * s1[3]) * 2.0;
    s2[5] = (c7 * s1[6] + c8 * s1[7]) * 2.0;
    s2[6] = (c3 * s1[3] - c4 * s1[2]) * 2.0;
    s2[7] = (c7 * s1[7] - c8 * s1[6]) * 2.0;

    double s3[8];
    s3[0] = (s2[4] + s2[5]) * 0.5;
    s3[1] = (s2[6] - s2[7]) * 0.5;
    s3[2] = s2[0];
    s3[3] = (s2[4] - s2[5]) * 0.5;
    s3[4] = s2[1];
    s3[5] = s2[2];
    s3[6] = s2[3];
    s3[7] = (s2[6] + s2[7]) * 0.5;

    double s4[8];
    s4[0] = s3[0];
    s4[1] = s3[1];
    s4[2] = s3[2];
    s4[3] = s3[4];
    s4[4] = s3[5];
    s4[5] = s3[3] * c9;
    s4[6] = s3[6];
    s4[7] = s3[7] * c9;

    double s5[8];
    s5[0] = (s4[2] + s4[3]) * 0.5;
    s5[1] = s4[0];
    s5[2] = (s4[4] + s4[5]) * 0.5;
    s5[3] = (s4[6] + s4[7]) * 0.5;
    s5[4] = (s4[2] - s4[3]) * 0.5;
    s5[5] = s4[1];
    s5[6] = (s4[4] - s4[5]) * 0.5;
    s5[7] = (s4[6] - s4[7]) * 0.5;

    double d[8];
    d[0] = (s5[0] + s5[1]) * 0.5;
    d[1] = (s5[2] - s5[3]) * 0.5;
    d[2] = (s5[2] + s5[3]) * 0.5;
    d[3] = (s5[4] - s5[5]) * 0.5;
    d[4] = (s5[4] + s5[5]) * 0.5;
    d[5] = (s5[6] + s5[7]) * 0.5;
    d[6] = (s5[6] - s5[7]) * 0.5;
    d[7] = (s5[0] - s5[1]) * 0.5;

    if(by_row) {
        for(int i = 0; i < 8; i++){
            if(d[i] < 0) {
                vec[rc * 8 + i] = (int16_t)((d[i] - 4) / 8);
            } else {
                vec[rc * 8 + i] = (int16_t)((d[i] + 4) / 8);
            }
        }
    }
    else {
        for(int i = 0; i < 8; i++){
            if(d[i] < 0){
                vec[i * 8 + rc] = (int16_t)((d[i] - 4) / 8);
            } else {
                vec[i * 8 + rc] = (int16_t)((d[i] + 4) / 8);
            }
        }
    }
}


/*
 *
 *  Start Of Image
 * 
 */
bool process_SOI(ifstream& jpegfile) {
    char c1, c2;
    jpegfile.read(&c1, 1);
    jpegfile.read(&c2, 1);

    /* test file format is jpeg */
    if (to_uint(c1) == TAG && to_uint(c2) == SOI) {
        if (debug)
            cout << "[SOI]" << endl;
        return 0;
    } else {
        cout << "[Error] invalid file format." << endl;
        return 1;
    }
}


/*
 *
 *  Define Quantization Table
 * 
 */
typedef uint8_t** qtable_t;

qtable_t init_qtable() {
    qtable_t qtable = new uint8_t*[4];
    for (int i = 0; i < 4; i++) {
        qtable[i] = new uint8_t[64];
    }
    return qtable;
}

void print_DQT(qtable_t qtable) {
    cout << "[Quantization Table]" << endl;
    int rows = 8, cols = 8;
    
    for (int i = 0; i < 4; i++) {
        cout << "  qid = " << i << endl;
        
        for (int r = 0; r < rows; r++) {
            cout << "    ";
            
            for (int c = 0; c < cols; c++) {
                printf("%2d ", (int)qtable[i][r * cols + c]);
            }
            cout << endl;
        }
    }
    cout << endl;
}

bool process_DQT(ifstream& jpegfile, qtable_t qtable) {
    unsigned int pointer = 0;
    char c1, c2;

    /* DQT Length */
    jpegfile.read(&c1, 1); pointer++;
    jpegfile.read(&c2, 1); pointer++;

    unsigned int len = to_num(c1, c2);

    if (debug)
        cout << "[DQT] len = " << len << endl;

    /* Fill in Quantization Table */
    while (pointer < len) {
        char c;
        jpegfile.read(&c, 1); pointer += 1;

        c = to_uint(c);
        unsigned int precision = c >> 4;
        unsigned int qid = c & 0x0F;

        for (int i = 0; i < 64; i++) {
            jpegfile.read(&c, 1); pointer += 1;
            qtable[qid][i] = (uint8_t)to_uint(c);
        }
    }

    return 0;
}


/*
 *
 *  Start Of Frame
 * 
 */
typedef struct hvq {
    unsigned int H; // horizontal sampling factor
    unsigned int V; // vertical sampling factor
    unsigned int Q; // quantization table id
} hvq_t;

typedef struct sof {
    unsigned int P; // precision
    unsigned int Y; // height
    unsigned int X; // width
    unsigned int N; // number of channels
    hvq_t* hvq;
    unsigned int hmax, vmax; // max value of H, V
} sof_t;

void print_SOF(sof_t& sof) {
    cout << "[SOF Information]" << endl;
    cout << "  precision = " << sof.P << endl;
    cout << "  height = " << sof.Y << endl;
    cout << "  width = " << sof.X << endl;
    cout << "  channels = " << sof.N << endl;
    for (unsigned int i = 0; i < sof.N; i++) {
        cout << "    H = " << sof.hvq[i].H << ", V = " << sof.hvq[i].V << ", Q = " << sof.hvq[i].Q << endl;
    }
    cout << endl;
}

bool process_SOF0(ifstream& jpegfile, sof_t& sof) {
    char c, c1, c2;

    /* SOF length */
    jpegfile.read(&c1, 1);
    jpegfile.read(&c2, 1);

    if (debug)
        cout << "[SOF0] len = " << to_num(c1, c2) << endl;

    /* precision */
    jpegfile.read(&c, 1);
    sof.P = to_uint(c);

    /* height */
    jpegfile.read(&c1, 1);
    jpegfile.read(&c2, 1);
    sof.Y = to_num(c1, c2);

    /* width */
    jpegfile.read(&c1, 1);
    jpegfile.read(&c2, 1);
    sof.X = to_num(c1, c2);

    /* number of channels */
    jpegfile.read(&c, 1);
    sof.N = to_uint(c) + 1; // id starts from 1 to 3, not from 0

    sof.hvq = new hvq_t[sof.N];
    sof.hmax = 0;
    sof.vmax = 0;

    for (unsigned int i = 1; i < sof.N; i++) { // channel id starts from 1
        /* channel id */
        jpegfile.read(&c, 1);
        unsigned int ci = to_uint(c);

        /* sample factors */
        jpegfile.read(&c, 1);
        c = to_uint(c);
        unsigned int h = c >> 4;
        unsigned int v = c & 0x0F;
        sof.hvq[ci].H = h;
        sof.hvq[ci].V = v;

        /* qtable id */
        jpegfile.read(&c, 1);
        sof.hvq[ci].Q = to_uint(c);

        /* update max value */
        if (h > sof.hmax)
            sof.hmax = h;
        if (v > sof.vmax)
            sof.vmax = v;
    }

    return 0;
}


/*
 *
 *  Define Huffman Table
 * 
 */
typedef pair<uint8_t, uint16_t> hkey_t;
typedef map<hkey_t, uint8_t> htable_t;

htable_t* init_htable() {
    htable_t* htable = new htable_t[4];
    return htable;
}

void print_DHT(htable_t* htable) {
    cout << "[Huffman Table]" << endl;
    
    for (unsigned int i = 0; i < 4; i++) {
        cout << "  hid = " << i << endl;
        htable_t::iterator iter;

        for (htable_t::iterator iter = htable[i].begin(); iter != htable[i].end(); iter++) {
            uint8_t length = iter->first.first;
            uint16_t symbol = iter->first.second;
            uint8_t code = iter->second;
            printf("    length = %2d, symbol = %04X, code = %02X\n", length, symbol, code);
        }
    }
    cout << endl;
}

bool process_DHT(ifstream& jpegfile, htable_t* htable) {
    unsigned int pointer = 0;
    char c, c1, c2;

    /* DHT length */
    jpegfile.read(&c1, 1); pointer++;
    jpegfile.read(&c2, 1); pointer++;

    unsigned int len = to_num(c1, c2);

    if (debug)
        cout << "[DHT] len = " << len << endl;

    while (pointer < len) {
        /* H table ID */
        jpegfile.read(&c, 1); pointer++;
        c = to_uint(c);
        unsigned int tc = c >> 4;
        unsigned int th = c & 0x0F;

        unsigned int hid = tc * 2 + th;

        /* counts */
        unsigned int counts[16];
        for (unsigned int i = 0; i < 16; i++) {
            jpegfile.read(&c, 1); pointer++;
            counts[i] = to_uint(c);
        }

        /* compute target symbols and get its corresponding codes */
        uint16_t symbol = 0;
        for (unsigned int i = 0; i < 16; i++) {
            for (unsigned int j = 0; j < counts[i]; j++) {
                jpegfile.read(&c, 1); pointer++; // code
                htable[hid].insert(
                    pair<hkey_t, uint8_t>(hkey_t(i + 1, symbol), to_uint(c))
                );
                symbol++;
            }
            symbol = symbol << 1;
        }
    }

    return 0;
}


/*
 *
 *  Start Of Scan
 * 
 */
typedef struct cda {
    unsigned int td;
    unsigned int ta;
} cda_t;

typedef struct sos {
    unsigned int ns;
    cda_t* cda;
    unsigned int ss;
    unsigned int se;
    unsigned int ah;
    unsigned int al;
} sos_t;

void print_SOS(sos_t& sos) {
    cout << "[SOS Information]" << endl;
    cout << "  channels = " << sos.ns << endl;
    for (unsigned int i = 0; i < sos.ns; i++) {
        cout << "    td = " << sos.cda[i].td << ", ta = " << sos.cda[i].ta << endl;
    }
    cout << "  Ss = " << sos.ss << endl;
    cout << "  Se = " << sos.se << endl;
    cout << "  Ah = " << sos.ah << endl;
    cout << "  Al = " << sos.al << endl;
    cout << endl;
}

bool process_SOS(ifstream& jpegfile, sos_t& sos) {
    unsigned int pointer = 0;
    char c, c1, c2;

    /* SOS length */
    jpegfile.read(&c1, 1); pointer++;
    jpegfile.read(&c2, 1); pointer++;
    unsigned int len = to_num(c1, c2);

    if (debug)
        cout << "[SOS] len = " << len << endl;

    while (pointer < len) {
        /* number of channels */
        jpegfile.read(&c, 1); pointer++;
        sos.ns = to_uint(c);

        sos.cda = new cda_t[sos.ns];

        /* information of color channels */
        for (unsigned int i = 0; i < sos.ns; i++) {
            jpegfile.read(&c, 1); pointer++;
            unsigned int cs = to_uint(c);

            jpegfile.read(&c, 1); pointer++;
            unsigned int tdta = to_uint(c);
            sos.cda[cs].td = tdta >> 4;
            sos.cda[cs].ta = tdta & 0x0F;
        }

        /* other information */
        jpegfile.read(&c, 1); pointer++;
        sos.ss = to_uint(c);

        jpegfile.read(&c, 1); pointer++;
        sos.se = to_uint(c);

        jpegfile.read(&c, 1); pointer++;
        unsigned int ahal = to_uint(c);
        sos.ah = ahal >> 4;
        sos.al = ahal & 0x0F;
    }

    return 0;
}


/*
 *
 *  Process Scan
 *
 */
typedef int16_t*** image_t;

image_t init_image(int d1, int d2, int d3) {
    image_t image = new int16_t**[d1];

    for (int i = 0; i < d1; i++) {
        image[i] = new int16_t*[d2];

        for (int j = 0; j < d2; j++) {
            image[i][j] = new int16_t[d3];
        }
    }

    return image;
}

void delete_image(image_t image, int d1, int d2, int d3) {
    for (int i = 0; i < d1; i++) {
        for (int j = 0; j < d2; j++) {
            delete[] image[i][j];
        }
        delete[] image[i];
    }
    delete[] image;
}

void to_bitmap(string filename, image_t rgb, sof_t& sof){
    bitmap_image bmp(sof.X, sof.Y);

    for (unsigned int py = 0; py < sof.Y; py++) {
        for (unsigned int px = 0; px < sof.X; px++) {
            bmp.set_pixel(
                px, py,
                (uint8_t)rgb[R][py][px],
                (uint8_t)rgb[G][py][px],
                (uint8_t)rgb[B][py][px]
            );
        }
    }

    size_t dot_i = filename.find_last_of(".");
    bmp.save_image(filename.substr(0, dot_i) + ".bmp");
}

typedef struct block_sxy {
    unsigned int stx; // start x
    unsigned int sty; // start y
    unsigned int sdx; // stride x
    unsigned int sdy; // stride y
} block_sxy_t;

image_t process_scan(ifstream& jpegfile, qtable_t qtable, sof_t& sof, htable_t* htable, sos_t& sos) {
    if (debug) {
        cout << "[Process Scan]" << endl;
        cout << "  get index" << endl;
    }

    /* get number of blocks in MCU, get start px and py of each block */
    unsigned int n_blocks = 0, ci = 0;
    vector<unsigned int> cid(n_blocks); // channel ids for each blocks in 1 MCU
    vector<block_sxy_t> b_sxy(n_blocks); // start index of blocks of each channel in 1 MCU

    for (unsigned int i = 1; i < sof.N; i++) { // id starts from 1
        n_blocks += sof.hvq[i].H * sof.hvq[i].V;

        cid.resize(n_blocks); // extend cid
        b_sxy.resize(n_blocks); // extend b_sxy
        unsigned int sci = ci; // start of current ci
        while(ci < n_blocks) {
            cid[ci] = i;
            b_sxy[ci].stx = ((ci - sci) % sof.hvq[i].H) * 8;
            b_sxy[ci].sty = ((ci - sci) / sof.hvq[i].H) * 8;
            b_sxy[ci].sdx = sof.hmax / sof.hvq[i].H;
            b_sxy[ci].sdy = sof.vmax / sof.hvq[i].V;
            ci++;
        }
    }

    unsigned int x_blocks = sof.hmax;
    unsigned int y_blocks = sof.vmax;
    unsigned int block_w = 8;
    unsigned int block_h = 8;

    /* get number of MCUs in this image */
    unsigned int MCU_w = sof.hmax * 8;
    unsigned int MCU_h = sof.vmax * 8;
    unsigned int x_MCUs = (sof.X - 1) / (MCU_w) + 1;
    unsigned int y_MCUs = (sof.Y - 1) / (MCU_h) + 1;
    unsigned int n_MCUs = x_MCUs * y_MCUs;

    if (debug) {
        cout << "  blocks in MCU = " << n_blocks << endl;
        cout << "  x MCUs = " << x_MCUs << endl;
        cout << "  y MCUs = " << y_MCUs << endl;
        cout << "  total MCUs = " << n_MCUs << endl;
        cout << "  read image..." << endl;
    }

    /* init image and buffer */
    image_t image = init_image(n_MCUs, n_blocks, 64);
    buffer_t buffer;

    /* prev dc, init to 0, index start from 1 */
    vector<uint16_t> prev_dc(sof.N, 0);

    /* read image */
    for (unsigned int i = 0; i < n_MCUs; i++) {
        for (unsigned int j = 0; j < n_blocks; j++) {

            bool end_of_block = false;

            /* qtable id, dc table id, ac table id */
            unsigned int qid = sof.hvq[cid[j]].Q;
            unsigned int dc = sos.cda[cid[j]].td;
            unsigned int ac = 2 + sos.cda[cid[j]].ta;

            for (unsigned int k = 0; k < 64; k++) {

                /* if end of block, then fill 0 until end */
                if (end_of_block) {
                    while (k < 64) {
                        image[i][j][k] = 0;
                        k++;
                    }
                    break;
                }

                /* get h table id */
                unsigned int hid = (k == 0) ? dc : ac;

                /* read bits and check htable */
                bool get_hkey = false;
                uint8_t len = 0;
                uint16_t symbol = 0;
                uint8_t code = 0;
                htable_t::iterator iter;
                do {
                    symbol = symbol << 1; // left shift 1 bit
                    symbol = symbol | read_bits(jpegfile, buffer, prev_dc, 1, true);
                    len += 1;

                    /* search from h table with length and symbol */
                    iter = htable[hid].find(hkey_t(len, symbol));
                    if (iter != htable[hid].end()) {
                        get_hkey = true;
                        code = iter->second;
                        //cout << "len = " << (int)len << ", symbol = " << symbol << ", code = " << (uint)code << endl;
                    }
                } while (!get_hkey);

                if (k == 0) {
                    int16_t delta = read_bits(jpegfile, buffer, prev_dc, code, false);
                    image[i][j][k] = prev_dc[cid[j]] + delta;
                    prev_dc[cid[j]] = image[i][j][k];

                    image[i][j][k] = image[i][j][k] * qtable[qid][k];

                } else {
                    if (code == 0x00) {
                        /* end of block */
                        image[i][j][k] = 0;
                        end_of_block = true;

                    } else {
                        /* run length coding */
                        uint8_t n_zeros = code >> 4;
                        uint8_t value = code & 0x0F;

                        k += n_zeros;
                        //cout << k << ", " << (int)n_zeros << ", " << (int)value << endl;

                        image[i][j][k] = read_bits(jpegfile, buffer, prev_dc, value, false);
                        image[i][j][k] = image[i][j][k] * qtable[qid][k];
                    }
                }
            }
            

            /* reverse zig-zag */
            int16_t uzz_block[64]; // un-zig-zagged block

            for (int k = 0; k < 64; k++) {
                uzz_block[k] = image[i][j][zzi[k]];
            }

            /* fast inverse discrete cosine transform */
            for (int r = 0; r < 8; r++) {
                fast_idct(uzz_block, r, true);
            }
            for (int c = 0; c < 8; c++) {
                fast_idct(uzz_block, c, false);
            }

            /* assign back to image */
            for (int k = 0; k < 64; k++) {
                image[i][j][k] = uzz_block[k];
            }
        }
    }

    if (debug) {
        cout << "  MCUs to YCbCr" << endl;
    }

    /* convert MCUs to YCbCr pixels */
    unsigned int channels = sos.ns;
    unsigned int height = MCU_h * y_MCUs;
    unsigned int width = MCU_w * x_MCUs;
    
    image_t ycbcr = init_image(channels, height, width);

    for (unsigned int i = 0; i < n_MCUs; i++) {
        /* find top left coord of MCU */
        unsigned int mcu_sx = (i % x_MCUs) * MCU_w;
        unsigned int mcu_sy = ((int)(i / x_MCUs)) * MCU_h;

        for (unsigned int j = 0; j < n_blocks; j++) {
            /* top left coord of block */
            unsigned int b_sx = mcu_sx + b_sxy[j].stx;
            unsigned int b_sy = mcu_sy + b_sxy[j].sty;

            /* channel */
            unsigned int c = cid[j] - 1; // channel id start from 1

            /* stride y */
            for (unsigned int sdy = 0; sdy < b_sxy[j].sdy; sdy++) {
                
                /* stride x */
                for (unsigned int sdx = 0; sdx < b_sxy[j].sdx; sdx++) {
                    
                    /* block */
                    for (unsigned int k = 0; k < 64; k++) {
                        /* find coord */
                        unsigned int px = b_sx + ((k % 8) * b_sxy[j].sdx + sdx);
                        unsigned int py = b_sy + ((k / 8) * b_sxy[j].sdy + sdy);

                        ycbcr[c][py][px] = image[i][j][k];
                    }
                }
            }
        }
    }

    /* free memory of image */
    delete_image(image, n_MCUs, n_blocks, 64);

    if (debug) {
        cout << "  YCbCr to RGB" << endl;
    }

    /* convert YCbCr to RGB */
    image_t rgb = init_image(channels, sof.Y, sof.X);

    for (int py = 0; py < sof.Y; py++) {
        for (int px = 0; px < sof.X; px++) {
            int16_t y = ycbcr[0][py][px];
            int16_t cb = ycbcr[1][py][px];
            int16_t cr = ycbcr[2][py][px];

            int16_t r = y + (int16_t)(1.402 * cr) + 128;
            int16_t g = y - (int16_t)(0.34414 * cb) - (int16_t)(0.71414 * cr) + 128;
            int16_t b = y + (int16_t)(1.772 * cb) + 128;

            rgb[R][py][px] = (r < 0) ? 0 : (r > 255) ? 255 : r;
            rgb[G][py][px] = (g < 0) ? 0 : (g > 255) ? 255 : g;
            rgb[B][py][px] = (b < 0) ? 0 : (b > 255) ? 255 : b;
        }
    }

    /* free memory of YCbCr */
    delete_image(ycbcr, channels, height, width);

    if (debug) {
        string rgb_str[3] = {"Red", "Green", "Blue"};

        for (int c = 0; c < 3; c++) {
            cout << "  " << rgb_str[c] << endl;

            for (int py = 0; py < 16; py++) {
                cout << "    ";

                for (int px = 0; px < 16; px++) {
                    printf("%3d ", rgb[0][py][px]);
                }
                cout << endl;
            }
            cout << endl;
        }
    }
    
    return rgb;
}

/*
 *
 *  End Of Image
 *
 */
bool process_EOI(bool &eoi) {
    if (debug)
        cout << "[EOI]" << endl;
    eoi = true;
    return 0;
}


/*
 *
 *  Other Tags
 *
 */
void process_other(ifstream& jpegfile, unsigned int opt) {
    switch (opt) {
        case 0xE0: case 0xE1: case 0xE2: case 0xE3:
        case 0xE4: case 0xE5: case 0xE6: case 0xE7:
        case 0xE8: case 0xE9: case 0xEA: case 0xEB:
        case 0xEC: case 0xED: case 0xEE: case 0xEF:
            if (debug)
                cout << "[APP" << to_uint(opt) % 16 << "] len = ";
            skip(jpegfile);
            break;
        
        case COM:
            if (debug)
                cout << "[COM] len = ";
            skip(jpegfile);
            break;
        
        case DRI:
            if (debug)
                cout << "[DRI] len = ";
            skip(jpegfile);
            USE_RST = true;
            break;

        default:
            printf("[Error] unknown tag: %04X\n", opt);
    }
}


/*
 *
 *  Main
 * 
 */
int main(int argc, char* argv[]) {
    if (argc < 2) {
        cout << "[Error] usage: ./jpeg-decoder [filename].jpg" << endl;
        return -1;
    }

    /* parse filename */
    string filename;
    for (int i = 0; i < argc; i++) {
        if (strcmp(argv[i], "--debug") == 0)
            debug = true;
        else
            filename = argv[i];
    }

    /* open jpeg */
    ifstream jpegfile;
    jpegfile.open(filename, ifstream::binary);
    if (!jpegfile.is_open()) {
        cout << "[Error] fail to open file: " << filename << endl;
        return -2;
    }

    /* read first tag */
    if (process_SOI(jpegfile))
        return -3;

    /* initialize */
    qtable_t qtable = init_qtable();
    sof_t sof;
    htable_t* htable = init_htable();
    sos_t sos;
    image_t rgb;

    char c1, c2;
    bool eoi = false; // end of image

    while(!eoi) {
        jpegfile.read(&c1, 1);
        jpegfile.read(&c2, 1);

        if (to_uint(c1) == TAG && to_uint(c2) != TAG) {

            unsigned int opt = to_uint(c2);

            switch(opt) {
                case DQT:
                    process_DQT(jpegfile, qtable);
                    break;
                
                case SOF0:
                    process_SOF0(jpegfile, sof);
                    break;
                
                case DHT:
                    process_DHT(jpegfile, htable);
                    break;
                
                case SOS:
                    process_SOS(jpegfile, sos);
                    if (debug) {
                        print_DQT(qtable);
                        print_SOF(sof);
                        print_DHT(htable);
                        print_SOS(sos);
                    }
                    rgb = process_scan(jpegfile, qtable, sof, htable, sos);
                    break;
                
                case EOI:
                    process_EOI(eoi);
                    break;
                
                default:
                    process_other(jpegfile, opt);
            }
        }
    }

    jpegfile.close();

    /* output bitmap */
    to_bitmap(filename, rgb, sof);

    return 0;
}