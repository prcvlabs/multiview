
#include "stdinc.hpp"

#include "labeled-slider.hh"

#include <QDial>
#include <QHBoxLayout>
#include <QLabel>
#include <QSlider>

#define This LabeledSlider

using namespace perceive;

static double to_percent(double val, double minv, double maxv)
{
   return std::clamp((val - minv) / (maxv - minv), 0.0, 1.0);
}

static QString to_qstr(string_view s) noexcept
{
   return QString::fromUtf8(s.data());
}

// ----------------------------------------------------------------------- pimpl

struct This::Pimpl
{
   This* parent{nullptr};
   double min_val{0};
   double max_val{1.0};
   unsigned steps{0};
   bool is_dial{false};

   QLabel* label{nullptr};
   QAbstractSlider* slider{nullptr};
   QDial* dial{nullptr};
   QSlider* vanilla{nullptr};

   Pimpl(double min_val,
         double max_val,
         unsigned steps,
         bool is_dial,
         This* in_parent)
       : parent(in_parent)
   {
      this->min_val = min_val;
      this->max_val = max_val;
      this->steps   = steps;
      this->is_dial = is_dial;
      this->parent  = in_parent;
   }
   void make_ui() noexcept;

   void set_number_steps() noexcept
   {
      slider->setMinimum(0);
      slider->setMaximum(steps - 1);
   }

   void set_label_text(double val) noexcept
   {
      label->setText(to_qstr(parent->label_render_fun(val)));
   }

   int slider_slots() const noexcept
   {
      return slider->maximum() - slider->minimum() + 1;
   }

   double value() const noexcept
   {
      const auto percent
          = to_percent(slider->value(), slider->minimum(), slider->maximum());
      return percent * (max_val - min_val) + min_val;
   }

   void set_value(double val) noexcept
   {
      const auto percent   = to_percent(val, min_val, max_val);
      const auto s_value   = percent * slider_slots() + slider->minimum();
      const auto old_state = slider->blockSignals(true);
      slider->setValue(s_value);
      slider->blockSignals(old_state);
      set_label_text(value());
   }
};

// --------------------------------------------------------------------- make ui

void This::Pimpl::make_ui() noexcept
{
   label = new QLabel{};

   if(is_dial)
      slider = dial = new QDial{};
   else
      slider = vanilla = new QSlider{Qt::Horizontal};

   label->setFixedWidth(60);

   if(!is_dial) vanilla->setTickPosition(QSlider::NoTicks);
   if(is_dial) dial->setWrapping(true);

   set_number_steps();
   set_label_text(0.0);

   if(is_dial) {
      auto layout = new QVBoxLayout{};
      layout->addWidget(slider);
      // layout->addWidget(label);
      parent->setLayout(layout);
   } else {
      auto layout = new QHBoxLayout{};
      layout->addWidget(slider);
      layout->addWidget(label);
      parent->setLayout(layout);
   }

   connect(
       slider, SIGNAL(valueChanged(int)), parent, SLOT(on_value_changed_(int)));
}

// ---------------------------------------------------------------- construction

This::This(const double min_val,
           const double max_val,
           const unsigned steps,
           const bool is_dial,
           QWidget* parent)
    : QWidget(parent)
    , pimpl_(new Pimpl(min_val, max_val, steps, is_dial, this))
{
   label_render_fun = [](double val) -> string { return format("{:8.2f}", val); };
   pimpl_->make_ui();
}

This::~This() { delete pimpl_; }

// ----------------------------------------------------------------------- value

double This::value() const noexcept { return pimpl_->value(); }

void This::set_value(double val) noexcept { pimpl_->set_value(val); }

bool This::block_signals(bool val) noexcept
{
   return pimpl_->slider->blockSignals(val);
}

// ------------------------------------------------------------------------ init

void This::init(const double min_val,
                const double max_val,
                const unsigned steps) noexcept
{
   const auto was_blocked = block_signals(true);

   pimpl_->min_val = std::min(min_val, max_val);
   pimpl_->max_val = std::max(min_val, max_val);
   pimpl_->steps   = steps;
   pimpl_->set_number_steps();

   block_signals(was_blocked);
}

// --------------------------------------------------------------------- getters

QLabel* This::get_label() noexcept { return pimpl_->label; }

unsigned This::n_steps() const noexcept { return pimpl_->steps; }

// ----------------------------------------------------------------------- slots

void This::on_value_changed_(int)
{
   pimpl_->set_label_text(value());
   emit value_changed(value());
}
