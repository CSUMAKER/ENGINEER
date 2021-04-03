#include "task_init.h"
#include "headfile.h"

int	main(void)
{
	serial_init(2, 115200);	//µ÷ÊÔ´®¿Ú

	makos_init();
	
	task_create(task_init, NULL, 1);
	
	makos_run();

	PRINT_ERROR("go back to main\r\n");
}
