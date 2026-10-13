"""
Ejecuta todas las pruebas de python/pruebas sin dependencias externas.

    python python/ejecutar_pruebas.py
"""
import importlib
import os
import sys
import traceback

CARPETA = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, CARPETA)

MODULOS = ['test_config', 'test_cinematica']


def main():
    total = fallas = 0
    for nombre in MODULOS:
        modulo = importlib.import_module(f'pruebas.{nombre}')
        print(f'\n== {nombre}')
        for prueba in [p for p in dir(modulo) if p.startswith('test_')]:
            total += 1
            try:
                getattr(modulo, prueba)()
                print(f'  [OK   ] {prueba}')
            except Exception:
                fallas += 1
                print(f'  [FALLA] {prueba}')
                traceback.print_exc()
    print(f'\nResumen: {total - fallas}/{total} pruebas superadas.')
    return 1 if fallas else 0


if __name__ == '__main__':
    sys.exit(main())
