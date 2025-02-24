
/*
    MD3 to OBJ Converter
    Converts Quake III MD3 models to Wavefront OBJ format.
    Supports single-file mode (one frame per OBJ) and merge mode (multiple MD3 files into one OBJ).
    Usage: md3toobj [options] input.md3 [output.obj | output_directory]
    Options:
      -flipUVs or -noFlipUVs
      -swapYZ or -noSwapYZ
      -merge (merge multiple MD3 files into one OBJ)
      
Created by: Christopher M. with the help of AI, and Github | Creatisoft https://www.creatisoft.com
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <errno.h>

/* MD3 file definitions (packed to match file layout) */
#pragma pack(push, 1)
typedef struct {
    char id[4];           // Should be "IDP3"
    int version;          // Should be 15
    char name[64];
    int flags;
    int numFrames;
    int numTags;
    int numSurfaces;
    int numSkins;
    int ofsFrames;
    int ofsTags;
    int ofsSurfaces;
    int ofsEnd;
} md3Header_t;

typedef struct {
    float mins[3];
    float maxs[3];
    float localOrigin[3];
    float radius;
    char name[16];
} md3Frame_t;

typedef struct {
    char id[4];           // Should be "IDP3"
    char name[64];
    int flags;
    int numFrames;
    int numShaders;
    int numVerts;
    int numTriangles;
    int ofsTriangles;
    int ofsShaders;
    int ofsST;            // Offset for texture coordinates
    int ofsVerts;
    int ofsEnd;
} md3Surface_t;

typedef struct {
    int indexes[3];
} md3Triangle_t;

typedef struct {
    float st[2];
} md3TexCoord_t;

typedef struct {
    short xyz[3];
    short normal;         // Encoded normal (angles)
} md3Vertex_t;

/* New: Tag structure – contains a name, origin and axis */
typedef struct {
    char name[64];
    float origin[3];
    float axis[3][3];
} md3Tag_t;
#pragma pack(pop)

#define MD3_VERSION 15
#define MD3_XYZ_SCALE (1.0f/64.0f)

/* Structure to hold surface data in memory */
typedef struct {
    md3Surface_t header;
    md3Triangle_t *triangles;
    md3TexCoord_t *texCoords;
    md3Vertex_t *vertices;  // Array of size: header.numVerts * header.numFrames
    int baseIndex;          // Global starting index for this surface’s vertices in OBJ output
} md3SurfaceData;

/* Structure to hold an MD3 file’s data (we use only the first frame for merging) */
typedef struct {
    md3Header_t header;
    md3SurfaceData *surfaces;
    int numSurfaces;
    /* New: store the full tag data if available */
    md3Tag_t *tags;
} md3FileData;

/* Global options (default: both enabled) */
int g_flipUVs = 1;
int g_swapYZ = 1;

/* Helper: get total file size (with error checking) */
long getFileSize(FILE *fp) {
    long curr = ftell(fp);
    if (curr < 0) {
        perror("ftell failed");
        return -1;
    }
    if (fseek(fp, 0, SEEK_END) != 0) {
        perror("fseek failed");
        return -1;
    }
    long size = ftell(fp);
    if (size < 0) {
        perror("ftell failed");
        return -1;
    }
    if (fseek(fp, curr, SEEK_SET) != 0) {
        perror("fseek failed");
        return -1;
    }
    return size;
}

/* Decode an MD3 encoded normal into a unit vector.
   Using the Quake III formula: each byte is scaled by (pi/128). */
void decodeNormal(short encoded, float *nx, float *ny, float *nz) {
    unsigned char lat = (encoded >> 8) & 0xFF;
    unsigned char lng = encoded & 0xFF;
    float latf = lat * (float)M_PI / 128.0f;
    float lngf = lng * (float)M_PI / 128.0f;
    *nx = cos(latf) * sin(lngf);
    *ny = sin(latf) * sin(lngf);
    *nz = cos(lngf);
}

/* Simple function to extract the basename (without path or extension) */
void getBasename(const char *path, char *basename, size_t size) {
    const char *p = strrchr(path, '/');
    if (!p) p = strrchr(path, '\\');
    if (p) p++; else p = path;
    strncpy(basename, p, size - 1);
    basename[size - 1] = '\0';
    char *dot = strrchr(basename, '.');
    if (dot) *dot = '\0';
}

/* Frees an array of surfaces and their allocated sub-objects */
void free_surfaces(md3SurfaceData *surfaces, int count) {
    for (int i = 0; i < count; i++) {
        free(surfaces[i].triangles);
        free(surfaces[i].texCoords);
        free(surfaces[i].vertices);
    }
    free(surfaces);
}

/* Reads a block of data from a given offset */
int read_from_offset(FILE *fp, long offset, void *buffer, size_t size, long fileSize) {
    if (offset < 0 || offset + size > (size_t)fileSize) {
        fprintf(stderr, "Invalid offset %ld or size %zu (fileSize=%ld).\n", offset, size, fileSize);
        return 0;
    }
    if (fseek(fp, offset, SEEK_SET) != 0) {
        fprintf(stderr, "fseek error: %s\n", strerror(errno));
        return 0;
    }
    if (fread(buffer, size, 1, fp) != 1) {
        fprintf(stderr, "fread error: %s\n", strerror(errno));
        return 0;
    }
    return 1;
}

/* Reads the MD3 header with extra validation against file size */
int read_md3_header(FILE *fp, md3Header_t *header, long fileSize) {
    if (fread(header, sizeof(md3Header_t), 1, fp) != 1) {
        fprintf(stderr, "Error reading MD3 header: %s\n", strerror(errno));
        return 0;
    }
    if (strncmp(header->id, "IDP3", 4) != 0 || header->version != MD3_VERSION) {
        fprintf(stderr, "Invalid MD3 file format or version.\n");
        return 0;
    }
    if ((long)header->ofsEnd > fileSize) {
        fprintf(stderr, "File appears truncated (ofsEnd exceeds file size).\n");
        return 0;
    }
    return 1;
}

/* Reads all surfaces from the MD3 file */
md3SurfaceData *read_md3_surfaces(FILE *fp, const md3Header_t *header, int *numSurfacesOut) {
    int numSurfaces = header->numSurfaces;
    long fileSize = getFileSize(fp);
    if (fileSize < 0) return NULL;
    md3SurfaceData *surfaces = (md3SurfaceData*) calloc(numSurfaces, sizeof(md3SurfaceData));
    if (!surfaces) {
        fprintf(stderr, "Memory allocation failed for surfaces.\n");
        return NULL;
    }
    if (fseek(fp, header->ofsSurfaces, SEEK_SET) != 0) {
        fprintf(stderr, "Error seeking to surfaces block.\n");
        free(surfaces);
        return NULL;
    }
    for (int s = 0; s < numSurfaces; s++) {
        long surfaceStart = ftell(fp);
        /* Read surface header */
        if (!read_from_offset(fp, surfaceStart, &surfaces[s].header, sizeof(md3Surface_t), fileSize)) {
            free_surfaces(surfaces, s);
            return NULL;
        }
        if (strncmp(surfaces[s].header.id, "IDP3", 4) != 0) {
            fprintf(stderr, "Invalid surface id at surface %d.\n", s);
            free_surfaces(surfaces, s + 1);
            return NULL;
        }
        /* Read triangles */
        int triSize = surfaces[s].header.numTriangles * sizeof(md3Triangle_t);
        surfaces[s].triangles = (md3Triangle_t*) malloc(triSize);
        if (!surfaces[s].triangles ||
            !read_from_offset(fp, surfaceStart + surfaces[s].header.ofsTriangles, surfaces[s].triangles, triSize, fileSize)) {
            fprintf(stderr, "Error reading triangles for surface %s.\n", surfaces[s].header.name);
            free_surfaces(surfaces, s + 1);
            return NULL;
        }
        /* Read texture coordinates */
        int tcSize = surfaces[s].header.numVerts * sizeof(md3TexCoord_t);
        surfaces[s].texCoords = (md3TexCoord_t*) malloc(tcSize);
        if (!surfaces[s].texCoords ||
            !read_from_offset(fp, surfaceStart + surfaces[s].header.ofsST, surfaces[s].texCoords, tcSize, fileSize)) {
            fprintf(stderr, "Error reading texture coordinates for surface %s.\n", surfaces[s].header.name);
            free_surfaces(surfaces, s + 1);
            return NULL;
        }
        /* Read vertices for all frames */
        int totalVerts = surfaces[s].header.numVerts * surfaces[s].header.numFrames;
        int vertSize = totalVerts * sizeof(md3Vertex_t);
        surfaces[s].vertices = (md3Vertex_t*) malloc(vertSize);
        if (!surfaces[s].vertices ||
            !read_from_offset(fp, surfaceStart + surfaces[s].header.ofsVerts, surfaces[s].vertices, vertSize, fileSize)) {
            fprintf(stderr, "Error reading vertices for surface %s.\n", surfaces[s].header.name);
            free_surfaces(surfaces, s + 1);
            return NULL;
        }
        /* Jump to the end of this surface block */
        if (fseek(fp, surfaceStart + surfaces[s].header.ofsEnd, SEEK_SET) != 0) {
            fprintf(stderr, "Error seeking to next surface.\n");
            free_surfaces(surfaces, s + 1);
            return NULL;
        }
    }
    *numSurfacesOut = numSurfaces;
    return surfaces;
}

/* Writes a single OBJ file for a given animation frame (single-file mode) */
int write_obj_frame(const md3Header_t *header, md3SurfaceData *surfaces, int numSurfaces, int frame, const char *outputName) {
    FILE *outFile = fopen(outputName, "w");
    if (!outFile) {
        fprintf(stderr, "Error opening output file %s: %s\n", outputName, strerror(errno));
        return 0;
    }
    /* Write object header */
    if (fprintf(outFile, "o %s\n", header->name) < 0) {
        fprintf(stderr, "Error writing to %s\n", outputName);
        fclose(outFile);
        return 0;
    }
    /* Write vertex positions (v) */
    for (int s = 0; s < numSurfaces; s++) {
        int numVerts = surfaces[s].header.numVerts;
        for (int v = 0; v < numVerts; v++) {
            int idx = frame * numVerts + v;
            md3Vertex_t vert = surfaces[s].vertices[idx];
            float x = vert.xyz[0] * MD3_XYZ_SCALE;
            float y = vert.xyz[1] * MD3_XYZ_SCALE;
            float z = vert.xyz[2] * MD3_XYZ_SCALE;
            if (g_swapYZ) {
                float temp = y; y = z; z = temp;
            }
            if (fprintf(outFile, "v %f %f %f\n", x, y, z) < 0) {
                fprintf(stderr, "Error writing vertex to %s\n", outputName);
                fclose(outFile);
                return 0;
            }
        }
    }
    /* Write texture coordinates (vt) – these do not change per frame */
    for (int s = 0; s < numSurfaces; s++) {
        int numVerts = surfaces[s].header.numVerts;
        for (int v = 0; v < numVerts; v++) {
            float u = surfaces[s].texCoords[v].st[0];
            float t = surfaces[s].texCoords[v].st[1];
            if (g_flipUVs) {
                t = 1.0f - t;
            }
            if (fprintf(outFile, "vt %f %f\n", u, t) < 0) {
                fprintf(stderr, "Error writing texcoord to %s\n", outputName);
                fclose(outFile);
                return 0;
            }
        }
    }
    /* Write vertex normals (vn) */
    for (int s = 0; s < numSurfaces; s++) {
        int numVerts = surfaces[s].header.numVerts;
        for (int v = 0; v < numVerts; v++) {
            int idx = frame * numVerts + v;
            md3Vertex_t vert = surfaces[s].vertices[idx];
            float nx, ny, nz;
            decodeNormal(vert.normal, &nx, &ny, &nz);
            if (g_swapYZ) {
                float temp = ny; ny = nz; nz = temp;
            }
            if (fprintf(outFile, "vn %f %f %f\n", nx, ny, nz) < 0) {
                fprintf(stderr, "Error writing normal to %s\n", outputName);
                fclose(outFile);
                return 0;
            }
        }
    }
    /* Write face definitions (f) */
    for (int s = 0; s < numSurfaces; s++) {
        if (fprintf(outFile, "g %s\n", surfaces[s].header.name) < 0) {
            fprintf(stderr, "Error writing group name to %s\n", outputName);
            fclose(outFile);
            return 0;
        }
        int base = surfaces[s].baseIndex;
        int numTris = surfaces[s].header.numTriangles;
        for (int t = 0; t < numTris; t++) {
            md3Triangle_t tri = surfaces[s].triangles[t];
            int i1, i2, i3;
            if (g_swapYZ) {
                i1 = base + tri.indexes[0];
                i2 = base + tri.indexes[1];
                i3 = base + tri.indexes[2];
            } else {
                i1 = base + tri.indexes[2];
                i2 = base + tri.indexes[1];
                i3 = base + tri.indexes[0];
            }
            if (fprintf(outFile, "f %d/%d/%d %d/%d/%d %d/%d/%d\n",
                        i1, i1, i1, i2, i2, i2, i3, i3, i3) < 0) {
                fprintf(stderr, "Error writing face data to %s\n", outputName);
                fclose(outFile);
                return 0;
            }
        }
    }
    fclose(outFile);
    return 1;
}

/* --- New Merge Mode Functions --- */

/* Reads a single MD3 file into an md3FileData structure.
   Now also reads tag data (if available). */
int load_md3_file(const char *filename, md3FileData *fileData) {
    FILE *fp = fopen(filename, "rb");
    if (!fp) {
        fprintf(stderr, "Error opening file %s: %s\n", filename, strerror(errno));
        return 0;
    }
    long fileSize = getFileSize(fp);
    if (fileSize < 0) {
        fclose(fp);
        return 0;
    }
    if (!read_md3_header(fp, &fileData->header, fileSize)) {
        fclose(fp);
        return 0;
    }
    /* If there are tags, read the first frame's tags */
    if (fileData->header.numTags > 0) {
        fileData->tags = (md3Tag_t*) malloc(fileData->header.numTags * sizeof(md3Tag_t));
        if (!fileData->tags) {
            fprintf(stderr, "Memory allocation failed for tags in %s\n", filename);
            fclose(fp);
            return 0;
        }
        if (!read_from_offset(fp, fileData->header.ofsTags, fileData->tags, fileData->header.numTags * sizeof(md3Tag_t), fileSize)) {
            fprintf(stderr, "Error reading tags for %s\n", filename);
            free(fileData->tags);
            fileData->tags = NULL;
        }
    } else {
        fileData->tags = NULL;
    }
    fileData->surfaces = read_md3_surfaces(fp, &fileData->header, &fileData->numSurfaces);
    fclose(fp);
    if (!fileData->surfaces) {
        return 0;
    }
    return 1;
}

/* Writes a merged OBJ file from multiple MD3 files (only first frame used).
   For each file, the vertex positions and normals are transformed using the
   first tag's transformation (if available). */
int write_merged_obj(md3FileData *files, int numFiles, const char *outputName) {
    FILE *outFile = fopen(outputName, "w");
    if (!outFile) {
        fprintf(stderr, "Error opening output file %s: %s\n", outputName, strerror(errno));
        return 0;
    }
    fprintf(outFile, "o MergedMD3\n");
    
    /* First pass: write vertex positions and compute global base indices */
    int globalIndex = 1;
    for (int f = 0; f < numFiles; f++) {
        md3FileData *mfile = &files[f];
        for (int s = 0; s < mfile->numSurfaces; s++) {
            mfile->surfaces[s].baseIndex = globalIndex;
            int numVerts = mfile->surfaces[s].header.numVerts;
            for (int v = 0; v < numVerts; v++) {
                int idx = 0 * numVerts + v;  // first frame only
                md3Vertex_t vert = mfile->surfaces[s].vertices[idx];
                float vx = vert.xyz[0] * MD3_XYZ_SCALE;
                float vy = vert.xyz[1] * MD3_XYZ_SCALE;
                float vz = vert.xyz[2] * MD3_XYZ_SCALE;
                float tx, ty, tz;
                if (mfile->tags) {
                    // Apply full transformation: rotated then translated
                    tx = mfile->tags[0].origin[0] +
                         mfile->tags[0].axis[0][0]*vx + mfile->tags[0].axis[0][1]*vy + mfile->tags[0].axis[0][2]*vz;
                    ty = mfile->tags[0].origin[1] +
                         mfile->tags[0].axis[1][0]*vx + mfile->tags[0].axis[1][1]*vy + mfile->tags[0].axis[1][2]*vz;
                    tz = mfile->tags[0].origin[2] +
                         mfile->tags[0].axis[2][0]*vx + mfile->tags[0].axis[2][1]*vy + mfile->tags[0].axis[2][2]*vz;
                } else {
                    tx = vx; ty = vy; tz = vz;
                }
                if (g_swapYZ) { float temp = ty; ty = tz; tz = temp; }
                if (fprintf(outFile, "v %f %f %f\n", tx, ty, tz) < 0) {
                    fprintf(stderr, "Error writing vertex to %s\n", outputName);
                    fclose(outFile);
                    return 0;
                }
            }
            globalIndex += numVerts;
        }
    }
    
    /* Second pass: write texture coordinates */  
    for (int f = 0; f < numFiles; f++) {
        md3FileData *mfile = &files[f];
        for (int s = 0; s < mfile->numSurfaces; s++) {
            int numVerts = mfile->surfaces[s].header.numVerts;
            for (int v = 0; v < numVerts; v++) {
                float u = mfile->surfaces[s].texCoords[v].st[0];
                float t = mfile->surfaces[s].texCoords[v].st[1];
                if (g_flipUVs) { t = 1.0f - t; }
                if (fprintf(outFile, "vt %f %f\n", u, t) < 0) {
                    fprintf(stderr, "Error writing texcoord to %s\n", outputName);
                    fclose(outFile);
                    return 0;
                }
            }
        }
    }
    
    /* Third pass: write vertex normals */  
    for (int f = 0; f < numFiles; f++) {
        md3FileData *mfile = &files[f];
        for (int s = 0; s < mfile->numSurfaces; s++) {
            int numVerts = mfile->surfaces[s].header.numVerts;
            for (int v = 0; v < numVerts; v++) {
                int idx = 0 * numVerts + v;
                md3Vertex_t vert = mfile->surfaces[s].vertices[idx];
                float nx, ny, nz;
                decodeNormal(vert.normal, &nx, &ny, &nz);
                if (mfile->tags) {
                    float nnx = mfile->tags[0].axis[0][0]*nx + mfile->tags[0].axis[0][1]*ny + mfile->tags[0].axis[0][2]*nz;
                    float nny = mfile->tags[0].axis[1][0]*nx + mfile->tags[0].axis[1][1]*ny + mfile->tags[0].axis[1][2]*nz;
                    float nnz = mfile->tags[0].axis[2][0]*nx + mfile->tags[0].axis[2][1]*ny + mfile->tags[0].axis[2][2]*nz;
                    nx = nnx; ny = nny; nz = nnz;
                }
                if (g_swapYZ) { float temp = ny; ny = nz; nz = temp; }
                if (fprintf(outFile, "vn %f %f %f\n", nx, ny, nz) < 0) {
                    fprintf(stderr, "Error writing normal to %s\n", outputName);
                    fclose(outFile);
                    return 0;
                }
            }
        }
    }
    
    /* Fourth pass: write face definitions for each surface */  
    for (int f = 0; f < numFiles; f++) {
        md3FileData *mfile = &files[f];
        for (int s = 0; s < mfile->numSurfaces; s++) {
            if (fprintf(outFile, "g %s\n", mfile->surfaces[s].header.name) < 0) {
                fprintf(stderr, "Error writing group name to %s\n", outputName);
                fclose(outFile);
                return 0;
            }
            int base = mfile->surfaces[s].baseIndex;
            int numTris = mfile->surfaces[s].header.numTriangles;
            for (int t = 0; t < numTris; t++) {
                md3Triangle_t tri = mfile->surfaces[s].triangles[t];
                int i1, i2, i3;
                if (g_swapYZ) {
                    i1 = base + tri.indexes[0];
                    i2 = base + tri.indexes[1];
                    i3 = base + tri.indexes[2];
                } else {
                    i1 = base + tri.indexes[2];
                    i2 = base + tri.indexes[1];
                    i3 = base + tri.indexes[0];
                }
                if (fprintf(outFile, "f %d/%d/%d %d/%d/%d %d/%d/%d\n",
                            i1, i1, i1, i2, i2, i2, i3, i3, i3) < 0) {
                    fprintf(stderr, "Error writing face data to %s\n", outputName);
                    fclose(outFile);
                    return 0;
                }
            }
        }
    }
    fclose(outFile);
    return 1;
}

/* --- End Merge Mode Functions --- */

/* Main: parses command-line arguments and selects mode */
int main(int argc, char *argv[]) {
    if (argc < 2) {
        printf("Usage: %s [options] input.md3 [output.obj | output_directory]\n", argv[0]);
        printf("  Options:\n");
        printf("    -flipUVs or -noFlipUVs\n");
        printf("    -swapYZ or -noSwapYZ\n");
        printf("    -merge (merge multiple MD3 files into one OBJ)\n");
        return 1;
    }
    
    int mergeMode = 0;
    /* For merge mode, use separate variables */
    char *mergeOutput = NULL;
    char **mergeInput = NULL;
    int numMergeInput = 0;
    
    /* For single-file mode */
    char *inputFile = NULL;
    char *outputFile = NULL;
    
    /* Parse command-line arguments */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-flipUVs") == 0) {
            g_flipUVs = 1;
        } else if (strcmp(argv[i], "-noFlipUVs") == 0) {
            g_flipUVs = 0;
        } else if (strcmp(argv[i], "-swapYZ") == 0) {
            g_swapYZ = 1;
        } else if (strcmp(argv[i], "-noSwapYZ") == 0) {
            g_swapYZ = 0;
        } else if (strcmp(argv[i], "-merge") == 0) {
            mergeMode = 1;
        } else if (mergeMode) {
            if (!mergeOutput && argv[i][0] != '-') {
                mergeOutput = argv[i];
            } else if (!mergeInput && argv[i][0] != '-') {
                mergeInput = &argv[i];
                numMergeInput = argc - i;
                break;
            }
        } else { /* Single file mode parsing */ 
            if (!inputFile && argv[i][0] != '-') {
                inputFile = argv[i];
            } else if (!outputFile && argv[i][0] != '-') {
                outputFile = argv[i];
            }
        }
    }
    
    if (mergeMode) {
        if (!mergeOutput || numMergeInput < 2) {
            fprintf(stderr, "Merge mode requires an output file followed by at least two input MD3 files.\n");
            return 1;
        }
        md3FileData *files = (md3FileData*) calloc(numMergeInput, sizeof(md3FileData));
        if (!files) {
            fprintf(stderr, "Memory allocation failed for merge files.\n");
            return 1;
        }
        int loaded = 0;
        for (int i = 0; i < numMergeInput; i++) {
            if (load_md3_file(mergeInput[i], &files[i])) {
                loaded++;
            } else {
                fprintf(stderr, "Failed to load %s\n", mergeInput[i]);
            }
        }
        if (loaded < 2) {
            fprintf(stderr, "At least two MD3 files must be loaded successfully for merge mode.\n");
            for (int i = 0; i < numMergeInput; i++) {
                if (files[i].surfaces) {
                    free_surfaces(files[i].surfaces, files[i].numSurfaces);
                }
                if (files[i].tags) {
                    free(files[i].tags);
                }
            }
            free(files);
            return 1;
        }
        if (!write_merged_obj(files, numMergeInput, mergeOutput)) {
            fprintf(stderr, "Failed writing merged OBJ file.\n");
        }
        for (int i = 0; i < numMergeInput; i++) {
            if (files[i].surfaces) {
                free_surfaces(files[i].surfaces, files[i].numSurfaces);
            }
            if (files[i].tags) {
                free(files[i].tags);
            }
        }
        free(files);
    } else {
        /* Single file mode */
        if (!inputFile) {
            fprintf(stderr, "No input file specified.\n");
            return 1;
        }
        FILE *inFile = fopen(inputFile, "rb");
        if (!inFile) {
            perror("Error opening input file");
            return 1;
        }
        long fileSize = getFileSize(inFile);
        if (fileSize < 0) {
            fclose(inFile);
            return 1;
        }
        md3Header_t header;
        if (!read_md3_header(inFile, &header, fileSize)) {
            fclose(inFile);
            return 1;
        }
        printf("Model: %s\nFrames: %d, Surfaces: %d\n", header.name, header.numFrames, header.numSurfaces);
        int numSurfaces = 0;
        md3SurfaceData *surfaces = read_md3_surfaces(inFile, &header, &numSurfaces);
        fclose(inFile);
        if (!surfaces) {
            return 1;
        }
        /* Compute a global base index for each surface */
        int globalIndex = 1;
        for (int s = 0; s < numSurfaces; s++) {
            surfaces[s].baseIndex = globalIndex;
            globalIndex += surfaces[s].header.numVerts;
        }
        char basename[256];
        if (outputFile) {
            getBasename(outputFile, basename, sizeof(basename));
        } else {
            getBasename(inputFile, basename, sizeof(basename));
        }
        int numFrames = header.numFrames;
        /* If more than one frame, output one OBJ per frame; otherwise, a single file */
        for (int frame = 0; frame < numFrames; frame++) {
            char outFilename[512];
            if (numFrames > 1) {
                snprintf(outFilename, sizeof(outFilename), "%s+%d.obj", basename, frame);
            } else {
                snprintf(outFilename, sizeof(outFilename), "%s.obj", basename);
            }
            printf("Writing frame %d to %s\n", frame, outFilename);
            if (!write_obj_frame(&header, surfaces, numSurfaces, frame, outFilename)) {
                fprintf(stderr, "Failed writing frame %d\n", frame);
            }
        }
        free_surfaces(surfaces, numSurfaces);
    }
    
    printf("Conversion completed successfully.\n");
    return 0;
}
