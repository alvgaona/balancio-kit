# Balancio-Kit

[![license_MIT](https://img.shields.io/github/license/udesa-ai/balancio-kit)](https://github.com/udesa-ai/balancio-kit/blob/main/LICENSE)
&nbsp;
[![stars](https://img.shields.io/github/stars/udesa-ai/balancio-kit)](https://github.com/udesa-ai/balancio-kit/stargazers)
&nbsp;
[![language_en](https://img.shields.io/badge/language-english-a0b0ba)](README_EN.md)
[![language_sp](https://img.shields.io/badge/spanish-1081c2)](README.md)

#### [README in English](README_EN.md)
----
Proyecto educativo de un robot :robot: de autobalanceo de ultra bajo costo, capaz de correr una red neuronal para mantener el equilibrio y de ser controlado de manera inalámbrica  :trackball:.

Desarrollado con fines didácticos para enseñar conceptos de RL, ML, AI y control.

<p align="center">
    <img src="resources/Balancio_0.6.jpg" height="200">
    <img src="resources/0.7a.jpeg" height="200">
    <img src="resources/balanciov3.jpg" height="200">
    <img src="resources/balancio_gif.gif" height="200">
    <img src="resources/Balancio_walle.jpg" height="200">
    <img src="resources/ensamble.jpg" height="200">
</p>



## Versiones
|                     | Estabilidad          | Facilidad de Armado  | Requiere I3D | Tiempo de [I3D](#configuración-de-impresión)| Plano de Impresión 3D| Foto de la versión                                 |
|:-------------------:|:--------------------:|:--------------------:|--------------|:----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|:--------------------------------------------------------------------------------------------------------------------------------------:|:--------------------------------------------------:|
| V 0.7 (Recomendado) | :star: :star: :star: | :star: :star: :star: | SI           | <ul style="list-style-type:none;"> <li>  Baterias 47m </li> <li> Cuerpo 2:36 h </li> </ul>                                                                                                       | <A HREF="https://github.com/AaronLoz/balancio-kit/tree/RL_1/Planos%20de%20Impresi%C3%B3n%203D/Version%200.7"> Versión 0.7 I3D</A>      | <img src="resources/Balancio_0.71.jpg" width="70"> |
| V 0.6 AKA:Wall-e    | :star: :star: :star: | :star:               | SI           | <ul style="list-style-type:none;"> <li> C_Inferior 5:25h </li> <li> C_Superior 5:11h </li> <li>P_inferior 58m</li> <li>P_superior 55m</li> <li> Cabeza 3:19 h </li> </ul> | <A HREF="https://github.com/AaronLoz/balancio-kit/tree/RL_1/Planos%20de%20Impresi%C3%B3n%203D/Versi%C3%B3n%200.6"> Versión 0.6 I3D</A> | <img src="resources/Balancio_0.5.jpg" width="60">  |
| V 0.5               | :star:               | :star: :star: :star: | SI           |<ul> <li>Aro 49m </li> <li> Cuerpo 4:42h </li>                                                                                                                                                                            | <A HREF="https://github.com/AaronLoz/balancio-kit/tree/RL_1/Planos%20de%20Impresi%C3%B3n%203D/versi%C3%B3n%200.5"> Versión 0.5 I3D</A> | <img src="resources/balanciov3.jpg" width="60">    |
    
### Configuración de Impresión 
- Espesor de boquilla: 0.4 mm
- Altura de Capa: 0.3 mm 
- Perimetros: 3 
- Capas Solidas Top y Bottom:  3 
- Infill: 20%


## Componentes
| Componente        | Imagen del Componente                                                                                   |
|:-----------------:|:------------------------------------------------------------------------------------------------------:|
| Placa base (estructura en impresion 3D) | <img src="resources/6D4F0806-B9EC-4AC4-8C3D-3CFF4A61EA02_1_105_c.jpeg" width="70">                                             |
| Microcontrolador NodeMCU ESP32 | <img src="resources/5C46DD2A-501A-4643-A7DC-159205664334_4_5005_c.jpeg" width="70">                                             |
| IMU MPU 6050 | <img src="resources/2CEC56FD-5F0F-4A58-AA3B-EB302D20FE31_4_5005_c.jpeg" width="70">                                             |
| 2 motorreductores de 6v con rueda "para Arduino" | <img src="resources/1701DEAB-A46C-4236-A983-E3C4270301A2_4_5005_c.jpeg" width="70">                                             |
| Puente-H L298N | <img src="resources/DA9AAA6E-92C4-4F05-AEAD-B6C834C8055B_4_5005_c.jpeg" width="70">                                             |
| 2 baterías 18650 con su correspondiente porta-pilas | <img src="resources/E30C2B73-9F67-4EAD-9B90-C8F0F0D67486_4_5005_c.jpeg" width="70">                                             |
| Cargador de baterias doble | <img src="resources/49A2AA11-48FB-4626-BE4D-3F666C9C9D7A_4_5005_c.jpeg" width="70">                                             |
| BMS FCD-2S-2 | <img src="resources/EC2FCAEF-AD10-4285-9255-656C09E6BECC_4_5005_c.jpeg" width="70">                                             |
| Switch | <img src="resources/173F8BFA-CFD3-4A2C-BAB3-88E42BB8DA0F_4_5005_c.jpeg" width="70">                                             |
| Cables DuPont | <img src="resources/5087FB85-E7A6-4EE5-AFD4-251F3EDD570A_4_5005_c.jpeg" width="70">                                             |
| Cables eléctricos bifilares (50 cm. c/u) | <img src="resources/6C6550A4-3079-4E1E-B1EE-0F5E0EAF8C07_1_105_c.jpeg" width="70">                                             |


<p align="center">
    <img src="resources/Balancio_plano.png" width="650">
</p>

## Plano Electrico 
<p align="center">
    <img src="resources/Plano_1.2.png" width="730">
</p>    
    
## Ensamblaje Mecánico 
1. Imprimir en 3D alguna de las [versiones](#versiones).
1. Encastrar piezas en su posición final.
1. Sostener los motores con precintos y/o pegamento. 
1. Conectar los componentes siguiendo el [plano electrico](#plano-electrico).  
     
## Instalación :floppy_disk:

En primer lugar, se debe clonar el repositorio. Esto se puede realizar tanto descargando el mismo
como un .ZIP, o ejecutando `git clone https://github.com/UDESA-AI/balancio-kit.git` en consola.

La instalación consta de 3 módulos principales: Microcontrolador, simulación y aplicación.
Éstos son fundamentales para el funcionamiento completo del proyecto, pero la instalación
de cada uno de ellos se puede realizar en distinto orden.
  
  
### Microcontrolador

<details open>
<summary>Instalación de la IDE</summary> 
Para programar y compilar el NodeMCU ESP32 usaremos la IDE 2 de Arduino. Para esto se
debe instalar la misma siguiendo los pasos que se especifican en el siguiente 
[link](https://www.arduino.cc/en/software) .
</details> 
<details open>
<summary>Configuración del Microcrontolador</summary> 
Una vez instalada la IDE, se debe habilitar el microcontrolador que vamos a usar.
Para esto se deben seguir los siguientes pasos:
    
1. En la IDE, ir a 'File' (Archivos) → 'Preferences' (Preferencias)
    
2. En el campo "Additional Boards Manager URLs", agregar lo siguiente: https://dl.espressif.com/dl/package_esp32_index.json. (Luego clickear 'OK').
3. Ir a 'Tools' → 'Board: ' → 'Boards Manager…' 
    
4. Buscar "esp32", e instalar "esp32 by Espressif Systems" (version 2.x) presionando el botón 'Install'.
    
5. Indicarle a la IDE que vamos a utilizar un esp32. Ir a 'Tools' → 'Board:' → 'ESP32 Arduino' → 'NodeMCU-32S'
    
6. En 'Tools' → 'Port', seleccionar el puerto correspondiente a donde está conectado el microcontrolador.

</details> 

<details open>
<summary>Librerias de Arduino</summary> 
Luego procederemos a instalar las librerías de arduino que vamos a utilizar:
    
   
- Para eso ir a 'Sketch' → 'Include Library' → 'Manage Libraries…'
    
    
- Buscar e instalar las siguientes librerias, especificando la versión correspondiente:
    - MPU6050 by Electronic Cats (version 1.0.0)
    - PS3 Controller Host by Jeffrey van Pernis (version 1.1.0)
    - EloquentTinyML by Simone Salerno (version 0.0.7)
    
    
</details> 
<details open>
<summary>Ejecución inicial</summary> 
Para comprobar la instalación, ejecutaremos un ejemplo de prueba:
- ir a 'File' → 'Examples' → 'WiFi' → 'WiFiScan'
- En el sketch generado, presionar el botón de carga ('Upload')  :calling:
- Si todo funcionó correctamente, debe aparecer un mensaje 'Done uploading' en la consola.

Posibles errores:
- Si no se puede cargar el programa al microcontrolador, intentar mantener presionado el botón "boot" presente en la placa, mientras se realiza la carga. Esto se debería realizar solo la primera vez.
</details> 

### Simulación
La simulación es opcional, no es necesaria para el funcionamiento y armado del robot. Se puede pasar directamente a la parte de calibración.
<details open>
 
La simulación corre en Python :snake:, y utiliza diversos paquetes. Para facilitar la instalación de los mismos, utilizaremos [Conda](https://docs.conda.io/en/latest/).

Se debe seguir con los siguientes pasos:

1. Para el uso e instalación de conda, descargaremos miniconda (también se puede instalar [Anaconda](https://docs.anaconda.com/anaconda/install/index.html)), siguiendo con los pasos que se especifican en el siguiente [link](https://docs.conda.io/en/latest/miniconda.html#installing).
2. Crearemos un 'Environment' de conda, donde alojaremos nuestros paquetes. 
   Esto se puede realizar tanto desde la consola (en el caso de haber descargado Miniconda) o desde una GUI (en caso de haber descargado Anaconda). Respectivamente:
    - Miniconda: Ejecutar el siguiente comando en la consola: `conda env create -f requirements.yml`. Donde `requierments.yml` es el [archivo](https://github.com/UDESA-AI/balancio-kit/blob/RL_1/requirements.yml) que se encuentra dentro del repositorio y ya fue descargado.
    - Anaconda: En la GUI de Anaconda: En la pestaña environments, hacer clik en import y especificar [archivo](https://github.com/UDESA-AI/balancio-kit/blob/RL_1/requirements.yml) en file

<p align="center">
    <img src="resources/env_anaconda.png" width="500">
</p>

3. Activar el environment creado, llamado **balancio**:
    - Miniconda: Ejecutar en terminal `conda activate balancio`
    - Anaconda: En la pestaña environments, hacer clik en el ambiente que se quiere activar
4. Dentro del environment activado, ejecutar el archivo [setup.py](https://github.com/UDESA-AI/balancio-kit/blob/RL_1/simulation/balancio_lib/setup.py):
    `python setup.py`
5. Probar la instalación, corriendo el siguiente [script](https://github.com/UDESA-AI/balancio-kit/blob/RL_1/simulation/pid.py):
    `python pid.py`
    </details> 
    

### Variables Físicas de la versión 0.7
#### Cuerpo
Masa sin ruedas: 0.244 kg



Posición del centro de masa con respecto al eje de los motores y el centro simétrico del cuerpo: 


- x = 1.55 
- y ~= 0 mm
- z = 31 mm

Inercia sin ruedas desde el centro de masa (kg.m2): 

- Ixx = 0.0006945
- Ixy ~= 0
- Iyy = 0.0006536
- Ixz = -0.000013447 
- Iyz ~= 0
- Izz = 0.0001937

#### Rueda
Masa de una rueda: 0.029 kg


Posición del centro de masa con respecto al eje de los motores y el borde del agarre: 
- x = 0 mm   
- y = 15.6 mm 
- z = 0 mm


Inercia de la rueda desde el centro de masa (kg.m2):  

- Ixx = 0.000011729 
- Ixy ~= 0  
- Iyy = 0.000021531 
- Ixz ~= 0 
- Iyz ~= 0 
- Izz = 0.000011729


### Aplicación 

 La aplicación está creada en <A HREF="https://appinventor.mit.edu/"> MIT App Inventor </A>.

Simplemente entrar al website e importar el .aia en <A HREF="Balancio-kit/app/app.aia"> App Balancio  </A>. Luego de esto, se puede usar la aplicación mediante bluethooth desde un celular.


## Calibración


Estas instrucciones asumen conocimiento del uso de la IDE arduino

<details open>
<summary>Calibración del IMU</summary>
1. Abrir `Balancio-kit/Mcu/Src/imu_calibration/imu_calibration.ino` con el IDE Arduino
    
2. Colocar el robot con la IMU paralela al piso y mantenerlo firme

3. Subir el programa a la placa y usar el monitor serial para obtener las compensaciones de la IMU

4. Modificar las compensaciones de giróscopo en el archivo `balancio-kit/mcu/src/main/config.h` en:
```c++
// IMU calibration parameters
#define X_ACCEL_OFFSET -1775
#define Y_ACCEL_OFFSET  756
#define Z_ACCEL_OFFSET  2706
#define X_GYRO_OFFSET   181
#define Y_GYRO_OFFSET   77
#define Z_GYRO_OFFSET   60
```
5. Para las compensaciones de Acelerómetro debe ser tenido en cuenta la gravedad, solo midiendo las compensaciones con la gravedad perpendicualar a esa direcci 
</details>  

<details open>
<summary>Calibración del angulo de equilibrio</summary>
1. Abrir `balancio-kit/mcu/src/main/main.ino`

2. Sostener el robot en la posición de equilibrio

3. Subir el programa a la placa y usar el monitor serial para obtener las compensaciones de angulo

4. Modificar en angulo de equilibrio en el archivo `balancio-kit/mcu/src/main/config.h` en la línea:
```c++
// Angle of (approximate) static equilibrium
#define STATIC_ANGLE -0.04 // Calibrated point
```
 </details>   
 
<details open>
<summary>Calibración de las constantes PID</summary>
    
1. Sacar el jumper de 12v en el driver 

2. Elegir parámetros PID
    
3. modificar las constantes del PID en el archivo `balancio-kit/mcu/src/main/config.h` en las líneas:
```c++
// PID Constants for pitch control
#define KP 2000
#define KI 22000
#define KD 20.0
```   

4. Probar las constantes, si se sacó el jumper se puede probar incluso con el cable conectado. **Cuidado al hacer esto!**


---
</details> 
     
Una vez configurado correctamente el robot, se pueden seleccionar distintos parametros de configuracion en el archivo correspondiente (`config.h`). 

Entre ellos, se puede seleccionar el tipo de controlador deseado para estabilizar el Balancio.

Por ejemplo, en caso de querer utilizar un controlador PID:
```c++
// Control algorithm type
#define CONTROL_ALGO "PID"
```

En caso de querer utilzar un agente de aprendizaje por refuerzo:
```c++
// Control algorithm type
#define CONTROL_ALGO "RL"
```
</details> 

---
## TODO

- [x] initial commit
- [x] Desarrollar aplicación bluetooth
- [x] Crear agente RL
- [x] Diseño mecánico
- [X] Publicar STL del diseño mecánico
- [ ] Publicar STEP del diseño mecánico
- [X] Crear diagrama electrónico
- [ ] Aclarar que datos levantar del imu calibration.



    
## Bugs conocidos
- [x] Wheel spins on startup



    
## Contribuciones
 
Las *pull requests* son bienvenidas, para cambios mayores, por favor abrir un *issue* para discutir los cambios deseados

## Licencia
[MIT](https://choosealicense.com/licenses/mit/)
