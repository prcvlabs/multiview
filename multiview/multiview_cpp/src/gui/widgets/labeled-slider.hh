
#pragma once

#include <functional>
#include <string>

#include <QLabel>
#include <QWidget>

class LabeledSlider : public QWidget
{
   Q_OBJECT

 private:
   struct Pimpl;
   Pimpl* pimpl_{nullptr};

 public:
   LabeledSlider(const double min_val = 0.0,
                 const double max_val = 1.0,
                 const unsigned steps = 100,
                 const bool is_dial   = false,
                 QWidget* parent      = nullptr);
   LabeledSlider(const LabeledSlider&) = delete;
   LabeledSlider(LabeledSlider&&)      = delete;
   virtual ~LabeledSlider();

   LabeledSlider& operator=(const LabeledSlider&) = delete;
   LabeledSlider& operator=(LabeledSlider&&) = delete;

   double value() const noexcept;
   void set_value(double val) noexcept;
   bool block_signals(bool val) noexcept;
   void init(const double min_val,
             const double max_val,
             const unsigned steps) noexcept;

   std::function<std::string(double)> label_render_fun;
   QLabel* get_label() noexcept;
   unsigned n_steps() const noexcept;

 signals:
   void value_changed(double value);

 private slots:
   void on_value_changed_(int value);
};
