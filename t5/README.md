#Tarea 5

**EL7008 - Procesamiento Avanzado de Imágenes**

*Rodrigo Muñoz* (rorro.mr@gmail.com)


## Requerimientos

* Ubuntu 14.04
* g++ 4.8.4
* make 3.81
* cmake 2.8
* opencv 2.4.8
* Boost Filesystem y System


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

**Entrenamiento**

Para realizar preprocesamiento, extracción de características y entrenamiento del clasificador.

```
$ cd /t5
$ ./t5 train
```
**Test con la base de datos**

Para obtener la respuesta del clasificador ante elementos de la base de datos.

```
$ cd /t5
$ ./t5 complete-test
```

## Calculo de transformada LBP

Uso: `$ ./t5 lbp imagen.jpg`


Ejemplo: 
```
$ cd t5
$ ./t5 make
$ ./t5 lbp db/male/cache2335957.jpg

```

## Test con imágenes

Uso: `$ ./t5 test imagen.jpg`. El archivo 'imagen.txt' con la posición de los ojos debe encontrarse en el mismo directorio.


Ejemplo: 
```
$ cd t5
$ ./t5 make
$ ./t5 test db/male/cache2335957.jpg

```

## Bases de datos

La bases de datos con imágenes de hombres y mujeres se guarda `./db/male` y `./db/female` respectivamente.

