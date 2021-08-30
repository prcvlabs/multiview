
#include "qt-helpers.hpp"

#include <QCheckBox>
#include <QFile>
#include <QFrame>
#include <QMessageBox>
#include <QTextStream>

#include "widgets/labeled-slider.hh"

namespace perceive
{
// ------------------------------------------------------------------- to-qimage

void to_qimage(const ARGBImage& im, QImage& qim)
{
   if(qim.size() != QSize(im.width, im.height))
      qim = QImage(im.width, im.height, QImage::Format_ARGB32);

   for(unsigned y = 0; y < im.height; ++y) {
      uint32_t* dst = reinterpret_cast<uint32_t*>(qim.scanLine(y));
      memcpy(dst, im.pixels + y * im.row_stride, im.row_bytes());
      uint32_t* end = dst + im.width;
      while(dst != end) *dst++ |= 0xff000000;
   }
}

void to_qimage(const GreyImage& im, QImage& qim)
{
   if(qim.size() != QSize(im.width, im.height))
      qim = QImage(im.width, im.height, QImage::Format_ARGB32);

   for(unsigned y = 0; y < im.height; ++y) {
      uint32_t* dst = reinterpret_cast<uint32_t*>(qim.scanLine(y));
      uint32_t* end = dst + im.width;
      uint8_t* src  = im.row_ptr(y);
      while(dst != end) {
         uint32_t g = *src++;
         *dst++     = 0xff000000u | (g << 16) | (g << 8) | (g << 0);
      }
   }
}

void to_qimage(const cv::Mat& im, QImage& qim)
{
   const auto cols = im.cols;
   const auto rows = im.rows;

   if(qim.size() != QSize(cols, rows))
      qim = QImage(cols, rows, QImage::Format_ARGB32);

   if(im.type() == CV_8UC3) {
      for(auto y = 0; y < rows; ++y) {
         const uint8_t* row = im.ptr(y); // BGR format
         uint32_t* dst      = reinterpret_cast<uint32_t*>(qim.scanLine(y));
         for(auto x = 0; x < cols; ++x) {
            uint8_t b = *row++;
            uint8_t g = *row++;
            uint8_t r = *row++;
            *dst++    = 0xff000000u | (r << 16) | (g << 8) | (b << 0);
         }
      }
   } else if(im.type() == CV_8UC1 || im.type() == CV_8U) {
      for(auto y = 0; y < rows; ++y) {
         const uint8_t* row = im.ptr(y);
         uint32_t* dst      = reinterpret_cast<uint32_t*>(qim.scanLine(y));
         for(auto x = 0; x < cols; ++x) {
            uint32_t v = *row++;
            *dst++     = 0xff000000u | (v << 16) | (v << 8) | (v << 0);
         }
      }
   } else {
      FATAL("Can only convert CV_8UC1 or CV_8UC3");
   }
}

// ----------------------------------------------------------------- from-qimage

static uint32_t convert_qt_alpha_channel(const uint32_t val) noexcept
{
   return (val & 0x00ffffffu) | (uint32_t(255 - alpha(val)) << 24);
}

void from_qimage(const QImage& qim, ARGBImage& im)
{
   const QImage* ptr = nullptr;
   QImage tmp;
   if(qim.format() == QImage::Format_ARGB32) {
      ptr = &qim;
   } else {
      tmp = qim.convertToFormat(QImage::Format_ARGB32);
      ptr = &tmp;
   }

   const int w = ptr->size().width();
   const int h = ptr->size().height();

   im.resize(w, h, w);
   for(auto y = 0; y < h; ++y) {
      const uint32_t* src = reinterpret_cast<const uint32_t*>(ptr->scanLine(y));
      uint32_t* dst       = im.row_ptr(y);
      for(auto x = 0; x < w; ++x) *dst++ = convert_qt_alpha_channel(*src++);
   }
}

// ------------------------------------------------------------------ to-qstring

QString to_qstr(string_view s) noexcept { return QString::fromUtf8(s.data()); }

string from_qstr(const QString& s) noexcept
{
   QByteArray utf8 = s.toUtf8();
   return string(utf8.constData());
}

// ------------------------------------------------------------- read asset file

string read_asset_file(const string& rsrc_name) noexcept(false)
{
   QFile f(":/gui.css");
   if(!f.open(QFile::ReadOnly | QFile::Text))
      throw std::runtime_error(
          format("failed to open resource '{}'", rsrc_name));
   QTextStream in(&f);
   return from_qstr(in.readAll());
}

// ------------------------------------------------------------- setting widgets

void block_signal_set(QCheckBox* cb, bool val) noexcept
{
   const bool was_blocked = cb->blockSignals(true);
   cb->setChecked(val);
   cb->blockSignals(was_blocked);
}

void block_signal_set(QRadioButton* rb, bool val) noexcept
{
   const bool was_blocked = rb->blockSignals(true);
   rb->setChecked(val);
   rb->blockSignals(was_blocked);
}

void block_signal_set(LabeledSlider* s, real val) noexcept
{
   const bool was_blocked = s->block_signals(true);
   s->set_value(val);
   s->block_signals(was_blocked);
}

void block_signal_set(QLineEdit* le, const string_view val) noexcept
{
   const bool was_blocked = le->blockSignals(true);
   le->setText(val.data());
   le->blockSignals(was_blocked);
}

void block_signal_set(QComboBox* c, const int val) noexcept
{
   const bool was_blocked = c->blockSignals(true);
   c->setCurrentIndex(val);
   c->blockSignals(was_blocked);
}

void block_signal_set_checked(QCheckBox* cb, bool val) noexcept
{
   block_signal_set(cb, val);
}
void block_signal_set_checked(QRadioButton* rb, bool val) noexcept
{
   block_signal_set(rb, val);
}

void block_signal_set_slider(LabeledSlider* ls, real val) noexcept
{
   block_signal_set(ls, val);
}

void block_signal_set_lineedit(QLineEdit* le, const string_view val) noexcept
{
   block_signal_set(le, val);
}

void block_signal_set_combobox(QComboBox* c, const int val) noexcept
{
   block_signal_set(c, val);
}

// ------------------------------------------------------------- widget to value

void widget_to_value(QCheckBox* cb, bool& val) noexcept
{
   val = cb->isChecked();
}

void widget_to_value(QRadioButton* rb, bool& val) noexcept
{
   val = rb->isChecked();
}

void widget_to_value(LabeledSlider* ls, real& val) noexcept
{
   val = ls->value();
}

void widget_to_value(LabeledSlider* ls, float& val) noexcept
{
   val = ls->value();
}

void widget_to_value(LabeledSlider* ls, int& val) noexcept
{
   val = ls->value();
}

void widget_to_value(QLineEdit* le, string& val) noexcept
{
   val = from_qstr(le->text());
}

void widget_to_value(QComboBox* c, int& val) noexcept
{
   val = c->currentIndex();
}

// ------------------------------------------------------------------ make-hline

QWidget* make_hline() noexcept
{
   auto line = new QFrame();
   line->setObjectName(QString::fromUtf8("hline"));
   line->setFixedHeight(4);
   line->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
   line->setFrameShape(QFrame::HLine);
   line->setFrameShadow(QFrame::Sunken);
   return line;
}

// ------------------------------------------------------------------ make-vline

QWidget* make_vline() noexcept
{
   auto line = new QFrame();
   line->setObjectName(QString::fromUtf8("vline"));
   line->setFixedWidth(4);
   line->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
   line->setFrameShape(QFrame::HLine);
   line->setFrameShadow(QFrame::Sunken);
   return line;
}

// ------------------------------------------- make horizontal/vertical expander

QWidget* make_horizontal_expander(int fixed_width) noexcept
{
   auto wgt = new QWidget{};
   wgt->setFixedHeight(0);
   if(fixed_width >= 0)
      wgt->setFixedWidth(fixed_width);
   else
      wgt->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
   return wgt;
}

QWidget* make_vertical_expander(int fixed_height) noexcept
{
   auto wgt = new QWidget{};
   wgt->setFixedWidth(0);
   if(fixed_height >= 0)
      wgt->setFixedHeight(fixed_height);
   else
      wgt->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
   return wgt;
}

// ------------------------------------------------------------------- new-label

QLabel* new_label(const string_view text, const char* style_sheet)
{
   auto lb = new QLabel(text.data());
   if(style_sheet) lb->setStyleSheet(style_sheet);
   return lb;
}

QLabel* new_header_label(const string_view text)
{
   return new_label(text, "font: bold italic;");
}

// ------------------------------------------------------- set message box width

void set_message_box_width(QMessageBox* msg_box, unsigned w) noexcept
{
   QSpacerItem* horizontalSpacer
       = new QSpacerItem(360, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);
   QGridLayout* layout = static_cast<QGridLayout*>(msg_box->layout());
   layout->addItem(
       horizontalSpacer, layout->rowCount(), 0, 1, layout->columnCount());
}

// --------------------------------------------------- set qform-min-label-width

void for_each_qform_label(QFormLayout* layout,
                          std::function<void(QWidget*)> f) noexcept
{
   QFormLayout::ItemRole role;
   int row   = 0;
   int count = layout->count();

   for(auto i = 0; i < count; ++i) {
      layout->getItemPosition(i, &row, &role);
      if(row >= 0 and role == QFormLayout::LabelRole) {
         QWidget* wgt = layout->itemAt(row, role)->widget();
         if(wgt != nullptr) f(wgt);
      }
   }
}

} // namespace perceive
