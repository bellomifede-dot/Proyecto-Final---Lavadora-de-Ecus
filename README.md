# Proyecto SCARA + Lavadora de ECUs

**Célula automatizada para la limpieza y manejo de Unidades de Control Electrónico (ECU)**

Este proyecto desarrolla una célula automatizada capaz de localizar, manipular y limpiar placas electrónicas (ECUs) mediante un robot SCARA y una unidad de lavado. El sistema busca reducir la intervención manual, homogeneizar el proceso y mejorar la trazabilidad de las piezas sometidas a mantenimiento o preparación previa a ensayos. 

<p align="center">
  <img src="Fotos y Videos/Fotos Readme/imagen1.png" alt="Placa ECU - vista" width="700" />
</p>

---
## Tabla de contenidos
- [Descripción](#descripción)
- [Objetivos](#objetivos)
- [Funcionamiento general](#funcionamiento-general)
- [Componentes principales](#componentes-principales)
---

## Descripción
El sistema consiste en una mesa de entrada donde se colocan las placas sucias, un subsistema de visión que localiza cada placa, un robot SCARA que las recoge y entrega a una caja de lavado. La caja realiza dos etapas: un rociado dirigido (para despegar suciedad superficial) y una inmersión demostrativa en una bandeja. Al finalizar, la placa se devuelve al punto de salida lista para inspección o uso. 

## Objetivos
- Automatizar la manipulación y limpieza de ECUs.
- Minimizar el manejo manual y la exposición del operario a solventes.
- Homogeneizar el proceso para asegurar repetibilidad y trazabilidad.
- Proveer una base técnica y documental para reproducir o adaptar la célula en otros entornos.

## Funcionamiento general
1. **Detección**: la cámara ubica las placas en la mesa mediante algoritmos de visión por computador.  
2. **Clasificación / Etiquetado (opcional)**: identificación del tipo de ECU y registro.  
3. **Agarre y transporte**: el robot SCARA toma la placa y la coloca en la estación de lavado.  
4. **Lavado en dos etapas**:
   - Rociado dirigido (elimina suciedad superficial).
   - Inmersión demostrativa en bandeja de limpieza.  
5. **Devolución**: la placa vuelve al puesto de salida para inspección y trazabilidad.

## Componentes principales
- **Robot SCARA** — para recogida y manipulación precisa.  
- **Subsistema de visión** — detección y posicionamiento.  
- **Caja de lavado** — sistema de rociado + inmersión.  
- **Controlador / PC maestro** — coordina visión, robot y secuencia de lavado.  
- **ECUs** — piezas a limpiar; sensibles a humedad/solventes y por tanto se deben seguir consideraciones de seguridad.

<p align="center">
  <a href="https://youtu.be/JoXS6hfcZ1g" target="_blank" rel="noopener noreferrer">
    <img src="https://img.youtube.com/vi/JoXS6hfcZ1g/hqdefault.jpg" alt="Demo - Lavadora de ECUs" width="720" />
  </a>
</p>

<p align="center"><em>Ver demo: Lavadora de ECUs</em></p>

