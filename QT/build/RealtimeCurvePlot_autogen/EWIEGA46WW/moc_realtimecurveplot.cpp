/****************************************************************************
** Meta object code from reading C++ file 'realtimecurveplot.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.4.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../realtimecurveplot.h"
#include <QtGui/qtextcursor.h>
#include <QScreen>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'realtimecurveplot.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.4.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
namespace {
struct qt_meta_stringdata_RealtimeCurvePlot_t {
    uint offsetsAndSizes[24];
    char stringdata0[18];
    char stringdata1[13];
    char stringdata2[1];
    char stringdata3[14];
    char stringdata4[11];
    char stringdata5[7];
    char stringdata6[23];
    char stringdata7[5];
    char stringdata8[13];
    char stringdata9[6];
    char stringdata10[23];
    char stringdata11[6];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(sizeof(qt_meta_stringdata_RealtimeCurvePlot_t::offsetsAndSizes) + ofs), len 
Q_CONSTINIT static const qt_meta_stringdata_RealtimeCurvePlot_t qt_meta_stringdata_RealtimeCurvePlot = {
    {
        QT_MOC_LITERAL(0, 17),  // "RealtimeCurvePlot"
        QT_MOC_LITERAL(18, 12),  // "refreshCurve"
        QT_MOC_LITERAL(31, 0),  // ""
        QT_MOC_LITERAL(32, 13),  // "onLegendClick"
        QT_MOC_LITERAL(46, 10),  // "QCPLegend*"
        QT_MOC_LITERAL(57, 6),  // "legend"
        QT_MOC_LITERAL(64, 22),  // "QCPAbstractLegendItem*"
        QT_MOC_LITERAL(87, 4),  // "item"
        QT_MOC_LITERAL(92, 12),  // "QMouseEvent*"
        QT_MOC_LITERAL(105, 5),  // "event"
        QT_MOC_LITERAL(111, 22),  // "onDisplayLengthChanged"
        QT_MOC_LITERAL(134, 5)   // "index"
    },
    "RealtimeCurvePlot",
    "refreshCurve",
    "",
    "onLegendClick",
    "QCPLegend*",
    "legend",
    "QCPAbstractLegendItem*",
    "item",
    "QMouseEvent*",
    "event",
    "onDisplayLengthChanged",
    "index"
};
#undef QT_MOC_LITERAL
} // unnamed namespace

Q_CONSTINIT static const uint qt_meta_data_RealtimeCurvePlot[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   32,    2, 0x08,    1 /* Private */,
       3,    3,   33,    2, 0x08,    2 /* Private */,
      10,    1,   40,    2, 0x08,    6 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 4, 0x80000000 | 6, 0x80000000 | 8,    5,    7,    9,
    QMetaType::Void, QMetaType::Int,   11,

       0        // eod
};

Q_CONSTINIT const QMetaObject RealtimeCurvePlot::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_RealtimeCurvePlot.offsetsAndSizes,
    qt_meta_data_RealtimeCurvePlot,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_stringdata_RealtimeCurvePlot_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<RealtimeCurvePlot, std::true_type>,
        // method 'refreshCurve'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onLegendClick'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<QCPLegend *, std::false_type>,
        QtPrivate::TypeAndForceComplete<QCPAbstractLegendItem *, std::false_type>,
        QtPrivate::TypeAndForceComplete<QMouseEvent *, std::false_type>,
        // method 'onDisplayLengthChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<int, std::false_type>
    >,
    nullptr
} };

void RealtimeCurvePlot::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<RealtimeCurvePlot *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->refreshCurve(); break;
        case 1: _t->onLegendClick((*reinterpret_cast< std::add_pointer_t<QCPLegend*>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QCPAbstractLegendItem*>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<QMouseEvent*>>(_a[3]))); break;
        case 2: _t->onDisplayLengthChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
        case 1:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 1:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< QCPAbstractLegendItem* >(); break;
            case 0:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< QCPLegend* >(); break;
            }
            break;
        }
    }
}

const QMetaObject *RealtimeCurvePlot::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *RealtimeCurvePlot::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_RealtimeCurvePlot.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int RealtimeCurvePlot::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
