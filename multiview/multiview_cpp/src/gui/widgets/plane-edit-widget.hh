
#pragma once

#include <QWidget>

class PlaneEditWidget : public QWidget
{
   Q_OBJECT

 private:
   struct Pimpl;
   Pimpl* pimpl_{nullptr};

 public:
   PlaneEditWidget();
   PlaneEditWidget(const PlaneEditWidget&) = delete;
   PlaneEditWidget(PlaneEditWidget&&)      = delete;
   virtual ~PlaneEditWidget();

   PlaneEditWidget& operator=(const PlaneEditWidget&) = delete;
   PlaneEditWidget& operator=(PlaneEditWidget&&) = delete;

   bool block_signals(const bool val) noexcept;

   int widget_id() const noexcept; // Id sent in signals

   // Getters/setters
   const std::string& name() const noexcept;
   int p3_type() const noexcept; // 0->X, 1->Y, 2->Z, otherwise->Z
   double p3_d() const noexcept;

   void set_name(const std::string_view s) noexcept;
   void set_p3_type(const int val) noexcept;
   void set_p3_d(const double val) noexcept;

   // Highlighting
   bool is_highlighted() const noexcept; // widget is highlighted
   void set_highlighted(const bool highlight) noexcept;

 signals:
   void on_delete_hit(int); // delete button was hit
   void on_changed(int);    // name, or plane parameters changed
   void on_selected(int);   // changed to highlighted
   void on_deselected(int); // changed to non-highlighted
   void on_clear_hit(int);  // The clear button was hit

 public slots:

   bool widgets_to_model();
   void model_to_widgets() noexcept;

 private slots:
   void on_select_action();
   void on_clear_action();
   void on_delete_action();
   void on_clicked();
};
