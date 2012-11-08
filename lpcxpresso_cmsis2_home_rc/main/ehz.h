#ifndef __EHZ_H
#define __EHZ_H

#define SERIAL_BUFFER_SIZE 12
#define SEARCH_PATTERN_LENGTH 6
#define EHZ_VALUE_LENGTH 11
#define EHZ_EXPECTED_DIGITS 10
#define EHZ_EXPECTED_DECIMAL_POSITION 6

void ehz_init();
void ehz_process_serial_data(uint8_t data);
uint8_t ehz_value_parsed();
uint32_t ehz_get_value();
uint32_t ehz_get_estimated_value();
uint32_t ehz_get_parsing_errors();


#endif /* end __EHZ_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/
