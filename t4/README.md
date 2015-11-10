#Tarea 3

**EL7008 - Procesamiento Avanzado de Imágenes**

*Rodrigo Muñoz* (rorro.mr@gmail.com)


## Requerimientos

* Ubuntu 14.04
* g++ 4.8.4
* make 3.81
* cmake 2.8
* opencv 2.4.8


## Compilación

**Permisos**

Permiso de ejecución al *script*.
```
$ cd t3
$ chmod +x t3
```

**Compilar**

Compilar código.
```
$ cd t3
$ ./t3 make
```

**Limpiar**

Elimina binarios y makefiles generados por CMake, junto con imágenes de resultados.

```
$ cd /t3
$ ./t3 clean
```

**Test**

Test con imágen de prueba.

```
$ cd /t3
$ ./t3 test
```

## Ejecución de ejemplos

Uso: `$ ./bin/t3 imagen imagen_ref`


Ejemplo: 
```
$ cd t3
$ ./t3 make
$ ./bin/t3 db/img01.jpg db/img02.jpg 

```

## Base de datos

La base de datos con imágenes de prueba se guarda en el directorio `./db`.

