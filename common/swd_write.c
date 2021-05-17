#include "swd_write.h"


int _write(int file, char *ptr, int len) {
	for(int i = 0; i < len; ++i) {
		ITM_Send_Char((*ptr++));
	}
	return len;
}
