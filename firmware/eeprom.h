#ifndef eeprom_h
#define eeprom_h

//Some helper macros
#define read_eeprom_byte(address) eeprom_read_byte ((const void *)address)
#define write_eeprom_byte(address,value_p) eeprom_write_byte (value_p, (void *)address)
#define update_eeprom_byte(address,value_p) eeprom_update_byte (value_p, (void *)address)

#define read_eeprom_array(address,value_p,length) eeprom_read_block ((void *)value_p, (const void *)address, length)
#define write_eeprom_array(address,value_p,length) eeprom_write_block ((const void *)value_p, (void *)address, length)
#define update_eeprom_array(address,value_p,length) eeprom_update_block ((const void *)value_p, (void *)address, length)

// Byte of memory to store infomation in:
#define EEPROM_UUID 0x01
#define EEPROM_I2CADDR 0x02


// Place to store the effect configuration
#define EEPROM_EFFECT 0x40


#endif
