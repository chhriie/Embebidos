# Proyecto Invernadero

Este repositorio contiene el desarrollo de un sistema de monitoreo y control para un **invernadero**, el cual recopila datos de sensores, los transmite a un nodo central (nodo edge) y acciona dispositivos deacuerdo a ciertas condiciones por medio del uso de esp32.

## Estructura del proyecto

```plaintext
/
├── actuadores/                # Código para el control de actuadores (actuadores.ino)
├── nodoEdge/                  # Nodo central encargado de procesar y almacenar datos
│   ├── nodoEdge.ino           # Código del nodo central
│   └── data/
│       └── config.json        # Configuración del nodo edge 
├── sensores/                  # Código de los nodos sensores (sensores.ino)
├── datos/                     # Datos generados por los sensores y almacenados en la microSD
│   └── ... (archivos CSV o TXT)
├── evidenciasInvernadero.pdf # Evidencia del funcionamiento del sistema 
├── presentacionInvernadero.pdf # Presentación del proyecto
└── README.md                  # Este archivo

