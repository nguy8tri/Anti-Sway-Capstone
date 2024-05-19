// Copyright 2024 Anti-Sway Team (Nguyen, Tri; Espinola, Malachi;
// Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)

#include <stdlib.h>
#include <string.h>


#include "matlabfiles.h"

#include "setup.h"

#include "record.h"


/* Resizable Array Constants */


// Default number of Files to remember
#define DEFAULT_NUM_FILES 3
// Default number of values for data arrays
#define DEFAULT_NUM_VALS 10
// Default resize factor
#define DEFAULT_RESIZE_FACTOR 2


/* Internal DataFile Representation */


/**
 * Represents a Data File
*/
typedef struct {
    MATFILE *file;
    int num_entries;
    char **entry_names;
    int num_vals;
    int vals_capacity;
    double **entry_values;
} DataFile_t;


/* Internal File Tracking Variables */


// The data files this module is handling
static DataFile_t *files = NULL;
// The number of files this module is handling
static int num_files = 0;
// The number of files this module can handle
static int capacity_files = 0;


/* Static Helper Functions */


/**
 * Deallocates the entire module
*/
static inline void DeallocateHelper();

/**
 * Reallocates the entry_values for a DataFile_t
 * 
 * @param f A pointer to the DataFile_t to resize
 * 
 * @return 0 iff success, negative upon error
 * 
 * @post for all 0 <= i <= f->num_entries, 
 * f->entry_values[i] is now double its capacity
 * from before, if, at the beginning of this function,
 * f->num_vals == f->vals->capacity
*/
static inline int ReallocateHelper(DataFile_t *f);


/* Function Definitions */


FileID_t OpenDataFile(char *name, char **entry_names, int num_entries) {
    // Make sure there's space for another file
    if (files == NULL) {
        files = (DataFile_t *) malloc(DEFAULT_NUM_FILES * sizeof(DataFile_t));
        if (files == NULL) {
            return EXIT_FAILURE;
        }
        capacity_files = DEFAULT_NUM_FILES;
    } else if (num_files == capacity_files) {
    	capacity_files *= DEFAULT_RESIZE_FACTOR;
        files = (DataFile_t *) realloc(files,
            capacity_files * sizeof(DataFile_t));
        if (files == NULL) {
            DeallocateHelper();
            return EXIT_FAILURE;
        }
    }

    if (num_entries < 1) {
        return EXIT_FAILURE;
    }

    // Copy the names into an allocated buffer
    char **entry_names_ = (char **) malloc(num_entries * sizeof(char *));
    if (entry_names == NULL) {
        return EXIT_FAILURE;
    }
    int i;
    for (i = 0; i < num_entries; i++) {
        int str_len = strlen(entry_names[i]) + 1;
        char *copy_name = (char *) malloc(str_len * sizeof(char));
        if (copy_name == NULL) {
            return EXIT_FAILURE;
        }
        snprintf(copy_name, str_len, entry_names[i]);
        entry_names_[i] = copy_name;
    }

    // Create data recorders
    double **entry_values_ = (double **) malloc(num_entries * sizeof(double *));
    if (entry_values_ == NULL) {
        return EXIT_FAILURE;
    }
    for (i = 0; i < num_entries; i++) {
        double *temp = (double *) malloc(DEFAULT_NUM_VALS * sizeof(double));
        if (temp == NULL) {
            return EXIT_FAILURE;
        }
        entry_values_[i] = temp;
    }

    DataFile_t *file = files + num_files;

    int err;
    file->file = openmatfile(name, &err);
    file->num_entries = num_entries;
    file->entry_names = entry_names_;
    file->num_vals = 0;
    file->vals_capacity = DEFAULT_NUM_VALS;
    file->entry_values = entry_values_;
    return num_files++;
}

int RecordData(FileID_t file, double data[], int data_length) {
    DataFile_t *f = &(files[file]);

    if (ReallocateHelper(f)) {
        DeallocateHelper();
        return EXIT_FAILURE;
    }

    int i;
    for (i = 0; i < f->num_entries; i++) {
        f->entry_values[i][f->num_vals] = data[i];
    }
    f->num_vals++;

    return EXIT_SUCCESS;
}

int SaveDataFiles() {
    DataFile_t *file;
    int err = EXIT_SUCCESS;
    for (file = files; file < files + num_files; file++) {
        int j;
        for (j = 0; j < file->num_entries; j++) {
            matfile_addmatrix(file->file,
                              file->entry_names[j],
                              file->entry_values[j],
                              1,
                              file->num_vals,
                              0);
        }
        if (matfile_close(file->file)) {
            err = EXIT_FAILURE;
        }
    }

    DeallocateHelper();
    return err;
}


/* Static Helper Functions */


static inline int ReallocateHelper(DataFile_t *f) {
    if (f->vals_capacity == f->num_vals) {
        int i;
        f->vals_capacity *= DEFAULT_RESIZE_FACTOR;
        for (i = 0; i < f->num_entries; i++) {
            f->entry_values[i] = (double *) realloc(f->entry_values[i],
                f->vals_capacity * sizeof(double));
            if (f->entry_values[i] == NULL) {
                return EXIT_FAILURE;
            }
        }
    }
    return EXIT_SUCCESS;
}

static inline void DeallocateHelper() {
    int j;
    DataFile_t *file = files;

    for (file = files; file < files + num_files; file++) {
        for (j = 0; j < file->num_entries; j++) {
            free(file->entry_names[j]);
            free(file->entry_values[j]);
        }
        free(file->entry_names);
        free(file->entry_values);
    }
    if (files != NULL) {
        free(files);
        files = NULL;
    }
    num_files = 0;
    capacity_files = 0;
}
