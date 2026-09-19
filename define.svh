`ifndef LAB3_DEFINE
`define LAB3_DEFINE

`define DATA_BITS 32
`define FILT_R 3
`define FILT_S 3

/* PE Define */
`define IFMAP_SIZE 8
`define FILTER_SIZE 8
`define PSUM_SIZE 32
`define IFMAP_SPAD_LEN 12
// Reduced from 48 to 16 to avoid DFF/BRAM resource explosion
`define FILTER_SPAD_LEN 16 
`define OFMAP_SPAD_LEN 4
`define IFMAP_INDEX_BIT 4
// Adjusted from 6 to 4 bits because max length is now 16 (2^4)
`define FILTER_INDEX_BIT 4 
`define OFMAP_INDEX_BIT 2
`define OFMAP_COL_BIT 5

/* PE Array Define*/
`define XID_BITS 5
`define YID_BITS 3
`define DEFAULT_XID  (2**`XID_BITS - 1)
`define DEFAULT_YID  (2**`YID_BITS - 1)
// Scaled down to 2x2 to fit ECP5-25F limit (28 DSPs max)
`define NUMS_PE_ROW 2 
`define NUMS_PE_COL 2 
`define DATA_SIZE 32
`define CONFIG_SIZE 10

`endif