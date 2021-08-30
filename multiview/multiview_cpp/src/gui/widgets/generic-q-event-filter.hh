
#pragma once

#include "stdinc.hpp"
#include <QObject>

class GenericQEventFilter : public QObject
{
   Q_OBJECT

 public:
   GenericQEventFilter(
       QObject* parent,
       std::function<bool(QObject* obj, QEvent* event)> event_filter_f);
   std::function<bool(QObject* obj, QEvent* event)> event_filter_f;

 protected:
   bool eventFilter(QObject* obj, QEvent* event);
};

/*
ui->displayWidget->installEventFilter(
new GenericQEventFilter(this, [&] (QObject *obj, QEvent *event) {
     if(event->type() == QEvent::Paint) {
       paint_display_widget(obj, event);
       return true;
     }
     return false;
   }));
*/
