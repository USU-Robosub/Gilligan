/****************************************************************************
** Meta object code from reading C++ file 'SubConsole.hpp'
**
** Created: Thu Feb 16 19:00:42 2012
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/SubConsole.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SubConsole.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SubConsole[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x08,
      32,   11,   11,   11, 0x08,
      53,   11,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_SubConsole[] = {
    "SubConsole\0\0readJoystickInput()\0"
    "handleRosCallbacks()\0joyConnect()\0"
};

const QMetaObject SubConsole::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_SubConsole,
      qt_meta_data_SubConsole, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SubConsole::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SubConsole::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SubConsole::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SubConsole))
        return static_cast<void*>(const_cast< SubConsole*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int SubConsole::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: readJoystickInput(); break;
        case 1: handleRosCallbacks(); break;
        case 2: joyConnect(); break;
        default: ;
        }
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
