// Copyright 2024 Anti-Sway Team (Nguyen, Tri; Espinola, Malachi;
// Tevy, Vattanary; Hokenstad, Ethan; Neff, Callen)

#ifndef RECORD_H_
#define RECORD_H_

typedef int FileID_t;

/**
 * Opens a data file
 * 
 * @param name The name of the file
 * @param entry_names The name of each entry
 * @param num_entries The number of entries
 * 
 * @return The file ID upon success, or negative upon failure
*/
FileID_t OpenDataFile(char *name, char **entry_names, int num_entries);

/**
 * Records data for each entry
 * 
 * @param name file the FileID_t to record upon
 * @param data The array of data to record (in order of the entries)
 * @param data_length The length of the data array
 * 
 * @param 0 iff success, negative upon failure
*/
int RecordData(FileID_t file, double data[], int data_length);

/**
 * Records all data into actual files, and closes all files
 * 
 * @return 0 iff success, negative upon failure
*/
int SaveDataFiles();

#endif  // RECORD_H_
