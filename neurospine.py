import os                                                                       # Per treballar amb carpetes i fitxers
import sys                                                                      # Per interactuar amb Python

_ROOT = os.path.dirname(os.path.abspath(__file__))                              # Guardem a _ROOT la carpeta on està aquest fitxer. Per fer-ho, primer busquem __file__ (fitxer actual), busquem la ruta completa (os.path.abspath) i només n'agafem la carpeta sense el fitxer (os.path.dirname)
if _ROOT not in sys.path:                                                       # si _ROOT no està entre les llistes internes de Python quan fem els imports
    sys.path.insert(0, _ROOT)                                                   # la inserim

_SCRIPTS = os.path.join(_ROOT, "scripts")                                       # Fem el mateix amb scripts. Ho podria haver fet amb src i altres carpetes. He decidit fer-ho amb _SCRIPTS perquè a dins hi ha neurospine_app i així la crido directe sempre sense haver de posar a davant cada vegada /scripts als imports. 
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)


def main():
    from qtpy import QtWidgets                                                  # EXPLICACIÓ MÉS AVALL
    from neurospine_app.main_window import NeuroSpineMainWindow                 # importem NeuroSpineMainWindow des de neurospine_app.main_window

    app = QtWidgets.QApplication(sys.argv)                                      # Per si volem fer servir un que no sigui Fusion. Ex: python neurospine.py -style windowsvista
    app.setStyle("Fusion")                                                      # És el més utilitzat per fer apps professionals. No diferències entre MAC i Windows.

    win = NeuroSpineMainWindow()                                                # Cridem a NeuroSpineMainWindow i s'encarrega de formar tota la app
    win.show()                                                                  # Mostrem tot el que hem creat a NeuroSpineMainWindow

    sys.exit(app.exec_())                                                       # Aquest bucle NO acaba fins que algú crida app.quit() o es tanca l'última finestra. És per mirar constantment el ratolí, zoom, clics, tecles...


if __name__ == "__main__":                                                      # Ho fem per si algú importa el mòdul, no vol que s'obri automàticament la finestra Qt. Vol tenir accés a main() per cridar-la quan li interessi.
    main()




"""
EXPLICACIÓ DE QtPy

Importem bindings (pont entre dos llenguatges). 
- Qt és una biblioteca feta amb C++ per fer aplicacions, menús, etc...   
- PyQt5 és el pont que permet a Python utilitzar Qt.     

RECORREGUT: JO - Python - QtPy (el cridem i s'encarrega de parlar amb PyQt5) - PyQt5 (binding o pont) - Qt (C++) - Finestra Windows.

Qt té diferents opcions com:
- QtWidgets     → finestres, botons, menús, etc.
- QtCore        → funcionament intern, senyals, temporitzadors...
- QtGui         → fonts, colors, imatges, esdeveniments...
...


i dins QtWidgets tenim: 
- QApplication   → aplicaicó gràfica
- QMainWindow    → finestra principal
- QPushButton    → botó
- QLabel         → text
- QWidget        → component/finestra bàsica
- QComboBox      → menú desplegable

"""