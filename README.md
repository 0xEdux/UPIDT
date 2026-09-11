# UPIDT - Métodos de sintonización de controladores PID

Este proyecto reúne funciones en MATLAB para calcular parámetros de controladores PID mediante los métodos de Ziegler-Nichols, además de utilidades para convertir expresiones simbólicas a funciones de transferencia y crear modelos de Simulink.

## Archivos principales

- `completeVars.m`: completa variables del sistema a partir de dos datos conocidos.
- `Metodo1zn.m`: método 1 de Ziegler-Nichols (curva de reacción).
- `Metodo2zn.m`: método 2 de Ziegler-Nichols (ganancia crítica).
- `sym2tf.m`: convierte una expresión simbólica en un objeto `tf`.
- `simulink_PID_creator.m`: crea un modelo de Simulink con un PID.
- `AdP.mlx`, `LGR.mlx`, `ZN.mlx`: ejemplos/plots interactivos en MATLAB Live Editor.

## Requisitos

- MATLAB con Control System Toolbox.
- Opcionalmente, Simulink para usar `simulink_PID_creator.m`.
- Para funciones simbólicas, es recomendable tener Symbolic Math Toolbox.

---

## 1) Completar variables del sistema

Archivo: `completeVars.m`

### Función

```matlab
[Wn, Xi, Mp, Ts] = completeVars(Wnd, Xid, Mpd, Tsd)
```

### Parámetros

- `Wnd`: frecuencia natural no amortiguada.
- `Xid`: amortiguamiento relativo.
- `Mpd`: sobreimpulso máximo.
- `Tsd`: tiempo de asentamiento.

### Cómo usar

Se pueden proporcionar dos de las cuatro variables y la función calcula las restantes cuando es posible.

Ejemplo:

```matlab
[Wn, Xi, Mp, Ts] = completeVars(-1, 0.5, 0.2, -1)
```

Esto intenta completar la frecuencia natural y el tiempo de asentamiento a partir del coeficiente de amortiguamiento y el sobreimpulso.

> La función exige que la combinación de datos sea suficiente; si no, lanza un error.

---

## 2) Método 1 de Ziegler-Nichols: curva de reacción

Archivo: `Metodo1zn.m`

### Función

```matlab
[Kp, Ti, Td, G] = Metodo1zn(tipo, fuente, datos, s)
```

### Parámetros

- `tipo`: tipo de controlador: `"P"`, `"PI"` o `"PID"`.
- `fuente`: origen de los datos: `"GRAFICA"` o `"FUNCION"`.
- `datos`: estructura con la información necesaria.
- `s`: variable de Laplace (por ejemplo, `s`).

### Uso con datos gráficos

Cuando se conoce la respuesta experimental, se entrega la estructura:

```matlab
syms s

datos.K = 2.5;
datos.L = 1.2;
datos.P = 4.8;

[Kp, Ti, Td, G] = Metodo1zn('PID', 'GRAFICA', datos, s);
```

### Uso con función de transferencia

Cuando la planta viene dada como expresión simbólica:

```matlab
syms s
datos.G_sym = 3 / (5*s^2 + 2*s + 1);

[Kp, Ti, Td, G] = Metodo1zn('PI', 'FUNCION', datos, s);
```

La función:

- convierte la expresión simbólica a `tf`;
- identifica ganancia `K`, tiempo muerto `L` y periodo `P`;
- aplica la tabla de Ziegler-Nichols;
- imprime el procedimiento paso a paso.

### Fórmulas aplicadas

Para el método 1, según el tipo de controlador:

- P:
  - `Kp = P / (K * L)`
- PI:
  - `Kp = 0.9 * P / (K * L)`
  - `Ti = L / 0.3`
- PID:
  - `Kp = 1.2 * P / (K * L)`
  - `Ti = 2 * L`
  - `Td = 0.5 * L`

---

## 3) Método 2 de Ziegler-Nichols: ganancia crítica

Archivo: `Metodo2zn.m`

### Función

```matlab
[Kp, Ti, Td, G] = Metodo2zn(tipo, fuente, datos, s)
```

### Parámetros

- `tipo`: `"P"`, `"PI"` o `"PID"`.
- `fuente`: `"GRAFICA"` o `"FUNCION"`.
- `datos`: estructura con la información requerida.
- `s`: variable de Laplace.

### Uso con datos gráficos

```matlab
datos.Kcr = 3.5;
datos.modoPeriodo = 'DIRECTO';
datos.Pcr = 2.4;

[Kp, Ti, Td, G] = Metodo2zn('PID', 'GRAFICA', datos, s);
```

O calculando el período a partir de dos picos:

```matlab
datos.Kcr = 3.5;
datos.modoPeriodo = 'DOS PICOS';
datos.tPico1 = 1.0;
datos.tPico2 = 3.4;

[Kp, Ti, Td, G] = Metodo2zn('PI', 'GRAFICA', datos, s);
```

### Uso con función de transferencia

```matlab
syms s
datos.G_sym = 2 / (s^3 + 4*s^2 + 5*s + 2);

[Kp, Ti, Td, G] = Metodo2zn('PID', 'FUNCION', datos, s);
```

La función:

- calcula el margen de ganancia de la planta;
- obtiene `Kcr` y la frecuencia crítica;
- determina `Pcr = 2*pi/Wcr`;
- aplica la regla de Ziegler-Nichols.

### Fórmulas aplicadas

- P:
  - `Kp = 0.5 * Kcr`
- PI:
  - `Kp = 0.45 * Kcr`
  - `Ti = Pcr / 1.2`
- PID:
  - `Kp = 0.6 * Kcr`
  - `Ti = 0.5 * Pcr`
  - `Td = 0.125 * Pcr`

---

## 4) Convertir una expresión simbólica a función de transferencia

Archivo: `sym2tf.m`

### Función

```matlab
sys = sym2tf(G_sym)
```

### Uso

```matlab
syms s
G = (2*s + 1) / (s^2 + 3*s + 2);
sys = sym2tf(G)
```

Esto devuelve un objeto `tf` de MATLAB listo para analizar o simular.

> Esta función es útil como paso previo para aplicar los métodos de Ziegler-Nichols sobre modelos simbólicos.

---

## 5) Crear un modelo de Simulink con PID

Archivo: `simulink_PID_creator.m`

### Función

```matlab
simulink_PID_creator(model_name, Gs, reference, Kp, Ki, Kd)
```

### Parámetros

- `model_name`: nombre del modelo a crear.
- `Gs`: planta en forma de `tf`.
- `reference`: valor del escalón de referencia.
- `Kp`, `Ki`, `Kd`: ganancias del controlador PID.

### Uso

```matlab
s = tf('s');
Gs = 1 / (s^2 + 3*s + 2);

simulink_PID_creator('PID_plantilla', Gs, 1, 1.2, 0.8, 0.1)
```

Esto crea automáticamente un modelo con:

- entrada tipo Step;
- sumador de error;
- ramas proporcional, integral y derivativa;
- suma del PID;
- planta;
- scope para visualizar la salida.

---

## 6) Ejemplo completo de uso

```matlab
clc; clear; close all;

syms s
G = 5 / (s^2 + 4*s + 5);

% Convertir a tf
Gtf = sym2tf(G);

% Metodo 1
[Kp1, Ti1, Td1, G1] = Metodo1zn('PID', 'FUNCION', struct('G_sym', G), s);

% Metodo 2
[Kp2, Ti2, Td2, G2] = Metodo2zn('PID', 'FUNCION', struct('G_sym', G), s);

% Crear modelo en Simulink
simulink_PID_creator('PID_Test', Gtf, 1, Kp1, Kp1/Ti1, Kp1*Td1)
```

---

## 7) Recomendaciones

- Usa el método 1 cuando tienes la curva de reacción del sistema.
- Usa el método 2 cuando puedes determinar la ganancia crítica y el período crítico.
- Para sistemas complejos o con retrasos, asegúrate de que la expresión mantenga la forma requerida por las funciones.
- Verifica siempre que la planta sea estable antes de aplicar algunos cálculos.

---

## 8) Notas

Las funciones están diseñadas para MATLAB y algunas requieren Toolbox específicos. Si el proyecto se ejecuta desde Octave, puede haber diferencias menores en la compatibilidad con funciones de control.

Si quieres, también puedo dejarte un README más formal orientado a entrega académica o un ejemplo de uso con una planta real específica.
