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
$ cd t2
$ chmod +x t2
```

**Compilar**

Compilar código.
```
$ cd t2
$ ./t2 make
```

**Limpiar**

Elimina binarios y makefiles generados por CMake.

```
$ cd /t2
$ ./t2 clean
```

##Ejecución de ejemplos

Uso: `$ ./bin/t2 lRBin lTBin cRBin cABin cBBin imagen.jpg`

Paramatros:
* `tlRBin`: Número de bins para radio de recta (r)
* `tlTBin`: Número de bins para angulo de recta (theta)
* `tcRBin`: Número de bins para radio de circulo (r)
* `tcABin`: Número de bins para posicion a del circulo (x-a)
* `tcBBin`: Número de bins para posicion b del circulo (x-b)

Ejemplo: 
```
$ cd t2
$ ./t2 make
$ ./bin/t2 100 180 50 50 50 db/img01.jpg

```

## Base de datos

La base de datos con imágenes de prueba se guarda en el directorio `./db`.

