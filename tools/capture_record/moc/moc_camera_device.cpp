/****************************************************************************
** Meta object code from reading C++ file 'camera_device.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../ffmpeg/camera_device.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'camera_device.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_CameraDevice_t {
    QByteArrayData data[17];
    char stringdata0[167];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CameraDevice_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CameraDevice_t qt_meta_stringdata_CameraDevice = {
    {
QT_MOC_LITERAL(0, 0, 12), // "CameraDevice"
QT_MOC_LITERAL(1, 13, 11), // "cameraState"
QT_MOC_LITERAL(2, 25, 0), // ""
QT_MOC_LITERAL(3, 26, 6), // "devKey"
QT_MOC_LITERAL(4, 33, 5), // "state"
QT_MOC_LITERAL(5, 39, 12), // "readyCapture"
QT_MOC_LITERAL(6, 52, 5), // "ready"
QT_MOC_LITERAL(7, 58, 16), // "previewAvailable"
QT_MOC_LITERAL(8, 75, 5), // "frame"
QT_MOC_LITERAL(9, 81, 12), // "saveImageSuc"
QT_MOC_LITERAL(10, 94, 8), // "filePath"
QT_MOC_LITERAL(11, 103, 13), // "recorderState"
QT_MOC_LITERAL(12, 117, 11), // "RecordState"
QT_MOC_LITERAL(13, 129, 10), // "recordTime"
QT_MOC_LITERAL(14, 140, 4), // "time"
QT_MOC_LITERAL(15, 145, 11), // "recordError"
QT_MOC_LITERAL(16, 157, 9) // "errorCode"

    },
    "CameraDevice\0cameraState\0\0devKey\0state\0"
    "readyCapture\0ready\0previewAvailable\0"
    "frame\0saveImageSuc\0filePath\0recorderState\0"
    "RecordState\0recordTime\0time\0recordError\0"
    "errorCode"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CameraDevice[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       7,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   49,    2, 0x06 /* Public */,
       5,    2,   54,    2, 0x06 /* Public */,
       7,    2,   59,    2, 0x06 /* Public */,
       9,    2,   64,    2, 0x06 /* Public */,
      11,    2,   69,    2, 0x06 /* Public */,
      13,    2,   74,    2, 0x06 /* Public */,
      15,    2,   79,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString, QMetaType::Bool,    3,    4,
    QMetaType::Void, QMetaType::QString, QMetaType::Bool,    3,    6,
    QMetaType::Void, QMetaType::QString, QMetaType::QImage,    3,    8,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,    3,   10,
    QMetaType::Void, QMetaType::QString, 0x80000000 | 12,    3,    4,
    QMetaType::Void, QMetaType::QString, QMetaType::LongLong,    3,   14,
    QMetaType::Void, QMetaType::QString, QMetaType::Int,    3,   16,

       0        // eod
};

void CameraDevice::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<CameraDevice *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->cameraState((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 1: _t->readyCapture((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 2: _t->previewAvailable((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< const QImage(*)>(_a[2]))); break;
        case 3: _t->saveImageSuc((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< const QString(*)>(_a[2]))); break;
        case 4: _t->recorderState((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< const RecordState(*)>(_a[2]))); break;
        case 5: _t->recordTime((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< qint64(*)>(_a[2]))); break;
        case 6: _t->recordError((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (CameraDevice::*)(const QString & , bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CameraDevice::cameraState)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (CameraDevice::*)(const QString & , bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CameraDevice::readyCapture)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (CameraDevice::*)(const QString & , const QImage & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CameraDevice::previewAvailable)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (CameraDevice::*)(const QString & , const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CameraDevice::saveImageSuc)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (CameraDevice::*)(const QString & , const RecordState & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CameraDevice::recorderState)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (CameraDevice::*)(const QString & , qint64 );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CameraDevice::recordTime)) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (CameraDevice::*)(const QString & , int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CameraDevice::recordError)) {
                *result = 6;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject CameraDevice::staticMetaObject = { {
    &QObject::staticMetaObject,
    qt_meta_stringdata_CameraDevice.data,
    qt_meta_data_CameraDevice,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *CameraDevice::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CameraDevice::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CameraDevice.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int CameraDevice::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void CameraDevice::cameraState(const QString & _t1, bool _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void CameraDevice::readyCapture(const QString & _t1, bool _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void CameraDevice::previewAvailable(const QString & _t1, const QImage & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void CameraDevice::saveImageSuc(const QString & _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void CameraDevice::recorderState(const QString & _t1, const RecordState & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void CameraDevice::recordTime(const QString & _t1, qint64 _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void CameraDevice::recordError(const QString & _t1, int _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
