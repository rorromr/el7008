#Tarea 1

**EL7008 - Procesamiento Avanzado de Imágenes**

*Rodrigo Muñoz* (rorro.mr@gmail.com)


## Requerimientos

* Ubuntu 14.04
* g++ 4.8.4
* make 3.81
* cmake 2.8


## Compilación

**Permisos**

Permiso de ejecución al *script*.
```
$ cd t1
$ chmod +x t1
```

**Compilar**

Compila programas.
```
$ cd t1
$ ./t1 make
```

**Limpiar**

Elimina imágenes filtradas, binarios y makefiles generados por CMake.

```
$ cd /t1
$ ./t1 clean
```

##Ejecución de ejemplos

Para ejecutar filtros con distintos imágenes de la base datos:

* Filtro mediana `$ ./t1 median`
* Equalización de histograma `$ ./t1 histeq`
* Filtro Gaussiano `$ ./t1 conv`

Cada ejecutable, en el directorio `./bin`, posee instrucciones de uso al ejecutalo sin parametros.

## Base de datos

La base de datos se guarda en el directorio `./db`, existen distintos subdirectorios:
* `./db/conv`: Para pruebas de filtros.
* `./db/histeq`: Para pruebas equalización de histograma.
* `./db/sp`: Para pruebas del filtro de mediana.

