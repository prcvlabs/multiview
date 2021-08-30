
#include "stdinc.hpp"

#include "plane-edit-widget.hh"

#include <QDoubleValidator>
#include <QIcon>

#include "gui/qt-helpers.hpp"

#define This PlaneEditWidget

using namespace perceive;

static std::atomic<int> id_counter{0};

// ----------------------------------------------------------------------- pimpl

struct This::Pimpl
{
   This* parent = nullptr;

   // Members
   int widget_id{0};
   string name;
   int p3_type{0}; // 0->X, 1->Y, 2->Z, otherwise->Z
   double d{0.0};
   bool is_highlighted{false};
   bool signals_blocked{false};

   // Widgets
   QPushButton* button_select{nullptr};
   QPushButton* button_clear{nullptr};
   QPushButton* button_delete{nullptr};
   QLineEdit* le_name{nullptr};
   QLineEdit* le_d{nullptr};
   QComboBox* combo_type{nullptr};

   Pimpl(This* parent_)
       : parent(parent_)
   {
      widget_id = ++id_counter;
   }

   void make_ui() noexcept;
   void set_highlight_stylesheet() noexcept;

   bool widgets_to_model() noexcept
   {
      bool changed     = false;
      const auto name_ = from_qstr(le_name->text());
      if(name_ != name) {
         name    = name_;
         changed = true;
      }

      if(p3_type != combo_type->currentIndex()) {
         p3_type = combo_type->currentIndex();
         changed = true;
      }

      const auto val = le_d->text().toDouble();
      if(val != d) {
         d       = val;
         changed = true;
      }

      if(changed and !signals_blocked) emit parent->on_changed(widget_id);
      return changed;
   }

   void model_to_widgets() noexcept
   {
      block_signal_set_lineedit(le_name, name);
      block_signal_set_lineedit(le_d, format("{:6.4f}", d));
      if(p3_type >= 0 and p3_type < 3)
         block_signal_set_combobox(combo_type, p3_type);
      else
         block_signal_set_combobox(combo_type, 2);

      le_name->setEnabled(is_highlighted);
      combo_type->setEnabled(is_highlighted);
      le_d->setEnabled(is_highlighted);
   }
};

// ---------------------------------------------------------------- construction

This::This()
    : pimpl_(new Pimpl(this))
{
   pimpl_->make_ui();
}

This::~This() { delete pimpl_; }

// --------------------------------------------------------------------- make ui

void This::Pimpl::make_ui() noexcept
{
   { // Create widgets
      button_select = new QPushButton{};
      button_clear  = new QPushButton{};
      button_delete = new QPushButton{};

      le_name    = new QLineEdit{};
      le_d       = new QLineEdit{};
      combo_type = new QComboBox{};

      button_select->setToolTip("Select.");
      button_clear->setToolTip("Clear selection.");
      button_delete->setToolTip("Delete.");

      le_name->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

      combo_type->addItem("X");
      combo_type->addItem("Y");
      combo_type->addItem("Z");

      le_d->setFixedWidth(80);
      le_d->setValidator(new QDoubleValidator(-99, 99, 5, parent));
   }

   { // Set Layout
      auto layout = new QHBoxLayout{};
      layout->setContentsMargins(0, 0, 0, 0);
      layout->setSpacing(0);
      layout->addWidget(button_select);
      layout->addWidget(le_name);
      layout->addWidget(combo_type);
      layout->addWidget(le_d);
      layout->addWidget(make_horizontal_expander());
      layout->addWidget(button_clear);
      layout->addWidget(button_delete);
      parent->setLayout(layout);
   }

   { // Initialize
      button_select->setFlat(true);
      button_clear->setFlat(true);
      button_delete->setFlat(true);
      // button_select->setStyleSheet("border: 1px;");
      // button_delete->setStyleSheet("border: 1px;");
      parent->setObjectName("planeEditWidget");
      set_highlight_stylesheet();
   }

   model_to_widgets();
   widgets_to_model();

   { // Wiring
      connect(
          button_select, SIGNAL(pressed()), parent, SLOT(on_select_action()));
      connect(button_clear, SIGNAL(pressed()), parent, SLOT(on_clear_action()));
      connect(
          button_delete, SIGNAL(pressed()), parent, SLOT(on_delete_action()));

      connect(
          le_name, SIGNAL(editingFinished()), parent, SLOT(widgets_to_model()));
      connect(combo_type,
              SIGNAL(currentIndexChanged(int)),
              parent,
              SLOT(widgets_to_model()));
      connect(
          le_d, SIGNAL(editingFinished()), parent, SLOT(widgets_to_model()));
   }
}

void This::Pimpl::set_highlight_stylesheet() noexcept
{
   if(parent->is_highlighted()) {
      button_select->setIcon(QIcon(":/icons/green-tick.png"));
      button_clear->setIcon(QIcon(":/icons/image.png"));
      button_delete->setIcon(QIcon(":/icons/circle-minus.png"));
      parent->setStyleSheet(R"V0G0N(
PlaneEditWidget {
   background-color: none;
   border-color: gray;
   border-width: 1px;
   border-style: outset;
}
)V0G0N");
   } else {
      button_select->setIcon(QIcon(":/icons/next-blue.png"));
      button_clear->setIcon(QIcon(":/icons/image-grey.png"));
      button_delete->setIcon(QIcon(":/icons/circle-minus-grey.png"));
      parent->setStyleSheet(R"V0G0N(
PlaneEditWidget {
   background-color: none;
   border-color: none;
   border-width: 1px;
}
)V0G0N");
   }

   parent->setAttribute(Qt::WA_StyledBackground);
}

// --------------------------------------------------------------- block signals

bool This::block_signals(const bool val) noexcept
{
   const auto ret          = pimpl_->signals_blocked;
   pimpl_->signals_blocked = val;
   return ret;
}

// --------------------------------------------------------------------- getters

int This::widget_id() const noexcept { return pimpl_->widget_id; }
const string& This::name() const noexcept { return pimpl_->name; }
int This::p3_type() const noexcept { return pimpl_->p3_type; }
double This::p3_d() const noexcept { return pimpl_->d; }

// --------------------------------------------------------------------- setters

void This::set_name(const string_view s) noexcept
{
   pimpl_->name = s;
   block_signal_set_lineedit(pimpl_->le_name, pimpl_->name);
   if(!pimpl_->signals_blocked) emit this->on_changed(widget_id());
}

void This::set_p3_type(const int val) noexcept
{
   pimpl_->p3_type = (val >= 0 and val <= 2) ? val : 2;
   block_signal_set_combobox(pimpl_->combo_type, pimpl_->p3_type);
   if(!pimpl_->signals_blocked) emit this->on_changed(widget_id());
}

void This::set_p3_d(const double val) noexcept
{
   pimpl_->d = val;
   pimpl_->model_to_widgets();
   if(!pimpl_->signals_blocked) emit this->on_changed(widget_id());
}

// -------------------------------------------------------------- is-highlighted

bool This::is_highlighted() const noexcept { return pimpl_->is_highlighted; }

void This::set_highlighted(const bool highlight) noexcept
{
   if(highlight != is_highlighted()) {
      pimpl_->is_highlighted = highlight;

      // Set the stylesheet
      pimpl_->set_highlight_stylesheet();

      pimpl_->model_to_widgets();
      // pimpl_->le_name->setEnabled(highlight);
      // pimpl_->combo_type->setEnabled(highlight);
      // pimpl_->le_d->setEnabled(highlight);

      // Emit on_select/deselect
      if(!pimpl_->signals_blocked) {
         if(highlight)
            emit this->on_selected(widget_id());
         else
            emit this->on_deselected(widget_id());
      }
   }
}

// ------------------------------------------------------- model to/from widgets

bool This::widgets_to_model() { return pimpl_->widgets_to_model(); }

void This::model_to_widgets() noexcept { return pimpl_->model_to_widgets(); }

// ------------------------------------------------------------ on delete action

void This::on_delete_action()
{
   if(!pimpl_->signals_blocked) emit this->on_delete_hit(widget_id());
}

// ------------------------------------------------------------- on clear action

void This::on_clear_action()
{
   if(!is_highlighted())
      set_highlighted(true);
   else
      emit this->on_clear_hit(widget_id());
}

// ------------------------------------------------------------ on select action

void This::on_select_action() { set_highlighted(!is_highlighted()); }

// ------------------------------------------------------------------ on clicked

void This::on_clicked()
{
   if(!is_highlighted()) { set_highlighted(true); }
}
