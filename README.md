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
