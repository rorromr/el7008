#Tarea 2

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
$ cd t2
$ chmod +x t2
```

**Compilar**

Compila programas.
```
$ cd t2
$ ./t2 make
```

**Limpiar**

Elimina imágenes filtradas, binarios y makefiles generados por CMake.

```
$ cd /t2
$ ./t2 clean
```

##Ejecución de ejemplos

Para ejecutar filtros con distintos imágenes de la base datos:

* Filtro mediana `$ ./t1 median`
* Equalización de histograma `$ ./t1 histeq`
* Filtro Gaussiano `$ ./t1 conv`

Cada ejecutable, en el directorio `./bin`, posee instrucciones de uso al ejecutalo sin parametros.

## Base de datos

La base de datos con imágenes de prueba se guarda en el directorio `./db`.

