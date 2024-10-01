# Control de cambios

1. 29/09/2024: 
    1.1. Se deshabilitaron las tareas `transmit_to_server_task()` y `lcd_task()`. Sus funciones fueron incluidas en la tarea `colect_data_task()`, para reducir el tiempo de transmisión de la posción actual y la actualización de la LCD. 
    1.2. Se redujo el tiempo de espera para la recepción de las colas `occupancy_queue` y `position_queue` en la tarea `colect_data_task()` de 500 ms a 100 ms. Esto reduce también el tiempo de espera para la recepción de la posición. 