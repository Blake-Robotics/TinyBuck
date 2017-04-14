/* tinyBuck i2c interface definitions
 *
 *  I2C interface allows configuration of the program
 *  running on the microprocessor and the execution
 *  of tasks.
 * 
 *  Note that all commands are processed as two bytes.
 *  Addresses below 0x20 are special and perform operations
 *  on the microprocessor.
 */

// Write commands (opcodes)
#define TB_RESET 0x0
#define TB_UPDATE 0x1
#define TB_SAVE 0x2
#define TB_SETI2C 0x13

// Synchronisation configuration
#define TB_SYNC_BEGIN 0x20

// Current Effect (and configuration), 
#define TB_EFFECT_CFG 0x40

// The standard effect configuration containts
// 1 identifyer byte, plus 15 configuration bytes.
#define TB_EFFECT_SIZE 16
typedef uint8_t effect_cfg[TB_EFFECT_SIZE];
