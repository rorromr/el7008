#Tarea 5

**EL7008 - Procesamiento Avanzado de Imágenes**

*Rodrigo Muñoz* (rorro.mr@gmail.com)


## Requerimientos

* Ubuntu 14.04
* g++ 4.8.4
* make 3.81
* cmake 2.8
* opencv 2.4.8
* Eigen 3
* Python (numpy, matplotlib)


## Compilación

**Permisos**

Permiso de ejecución al *script*.
```
$ cd t5
$ chmod +x t5
```

**Compilar**

Compilar código.
```
$ cd t5
$ ./t5 make
```

**Limpiar**

Elimina binarios y makefiles generados por CMake, junto con imágenes de resultados.

```
$ cd /t5
$ ./t5 clean
```

**Test**

Test con imágen de prueba.

```
$ cd /t5
$ ./t5 test
```

**Gráfico curva ROC**

Realiza un gráfico de la curva ROC usando `roc.csv` (requiere Python, numpy y matplotlib).

```
$ cd /t5
$ ./t5 plot
```

## Ejecución de ejemplos

Uso: `$ ./bin/t5 imagen imagen_ground_truth`


Ejemplo: 
```
$ cd t5
$ ./t5 make
$ ./bin/t5 ./db/0024.jpg ./db/0024.bmp

```

## Base de datos

La base de datos con imágenes de prueba se guarda en el directorio `./db`.

