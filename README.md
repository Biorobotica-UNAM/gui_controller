# gui_controller

## Configuracion local

El GUI usa la variable de entorno `JUSTINA_WS` para ubicar el workspace de Justina.
Define esta variable antes de abrir el GUI:

```bash
export JUSTINA_WS="$HOME/Justina"
```

Para dejarla permanente, agrega esa linea a tu `~/.bashrc` y recarga la terminal:

```bash
source ~/.bashrc
```

Verifica que apunte al workspace correcto:

```bash
echo "$JUSTINA_WS"
ls "$JUSTINA_WS/install/setup.bash"
```

El archivo `gui_controller/config.yaml` usa ese valor por medio del alias
`{justina_ws}`:

```yaml
aliases:
  justina_ws: "$JUSTINA_WS"
```

Si `JUSTINA_WS` no esta definida, el GUI mostrara un error antes de lanzar Surge
o Supervisor.

## Pestañas de demostracion

- **Lanzamiento** Surge y Supervisor.
- **Acciones** permite enviar al executor las acciones registradas en
  `executor_worker`, con un campo de argumento y feedback.
- **Hardware** reutiliza las rutinas `test_head`, `test_torso` y `test_arms`;
  tambien ofrece posiciones de torso/cabeza, las poses de brazo ya definidas
  por el proyecto y abrir/cerrar los grippers.
- **Demostraciones** incluye una presentacion, una coreografia breve,
  "Conoce a Justina" (presentacion, saludo con una mano, reconocimiento del
  nombre del visitante y respuesta personalizada), navegacion a una ubicacion
  conocida, avanzar/regresar una distancia, movilidad de la base y un flujo
  GPSR de un solo comando. El destino desplegable carga las ubicaciones del archivo
  `locations_manager/config/known_locations.yaml`.

Antes de ejecutar acciones manuales o de demostracion, Justina anuncia por voz
en ingles lo que hara. En la presentacion y la coreografia tambien anuncia los
movimientos que ejecutara. La presentacion usa directamente los servicios de
torso, cabeza y brazos para avanzar tras confirmar cada movimiento, sin cambiar
las rutinas de prueba `test_head`, `test_torso` ni `test_arms`. La accion manual
`say` envia el texto escrito al servicio de voz y espera su respuesta antes de
rehabilitar los controles; el texto se reproduce literalmente.
La demo de movilidad anuncia cada giro y desplazamiento y espera que cada
objetivo de navegacion termine antes de pasar al siguiente. Avanzar y regresar
usan la distancia seleccionada; regresar coloca el objetivo detras de la pose
actual y orienta a Justina en sentido contrario.

Los controles que mueven el robot se habilitan solo cuando
`/supervisor/supervisor_plan/active` confirma que el planificador esta inactivo.
La demo GPSR escucha y muestra las metas antes de pedir confirmacion; al
aceptarlas, `goals_to_clips` reinicia el contexto de CLIPS y comienza el plan.
Para evitar dos escuchas concurrentes, no uses esa demo cuando Surge se lanzo
con el perfil GPSR: ese perfil conserva su flujo existente, incluido el paso
`continue`.

Cuando el comando de Surge termina, el GUI limpia el estado de las acciones y
demostraciones en curso, restaura sus controles y habilita de nuevo el boton de
lanzamiento para poder iniciar Surge otra vez.

La casilla de Lanzamiento agrega `blank_map:=true` a Surge: carga los mapas
en blanco existentes usando la navegacion AMCL habitual (no inicia SLAM).
El valor predeterminado sigue cargando los mapas `lab_2026`; el perfil
Restaurant tambien selecciona el mapa en blanco.
El objetivo al frente usa la transformacion actual `map -> base_link`, calcula
un punto a la distancia elegida y lo publica en `/nav_control/goal`, el
mismo destino que usa RViz con **2D Nav Goal**. La navegacion pasa por la
planificacion normal de Surge, no por el controlador mapless experimental.
El GUI permite solicitar detener esa navegacion desde el topico de stop del
controlador normal.
