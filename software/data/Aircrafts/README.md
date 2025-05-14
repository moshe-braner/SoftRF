% ogn2cdb

# WHERE TO GET UP TO DATE OGN DDB DATA

ddb.glidernet.org/download - save as a text file

# OGN2CDB

This is a tool for converting the OGN DDB (downloaded as plain text) into the .cdb format (for SoftRF and SkyView).

# USAGE

In a Windows command line, type "ogn2cdb filename.txt".  Or, drag and drop the text file onto an icon for the EXE file.  Output will be in "ogn.cdb".

# SOURCE

	Author:     Richard James Howe
	Repository: <https://github.com/howerj/cdb>

Adapted for OGN DDB by Moshe Braner.  Empty records are skipped.

The full functionality of the original is retained, e.g.:

**-c**  *file.cdb* : run in create mode

**-d**  *file.cdb* : dump the database

**-s**  *file.cdb* : print statistics about the database

**-V**  *file.cdb* : validate database

**-q**  *file.cdb key record-number* : query the database for a key, with an optional record
