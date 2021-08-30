
#include "generic-q-event-filter.hh"

GenericQEventFilter::GenericQEventFilter(
    QObject* parent,
    std::function<bool(QObject* obj, QEvent* event)> event_filter_f)
    : QObject(parent)
    , event_filter_f(event_filter_f)
{}

bool GenericQEventFilter::eventFilter(QObject* obj, QEvent* event)
{
   return this->event_filter_f(obj, event);
}
