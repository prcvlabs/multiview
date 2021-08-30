
#pragma once

// Other QT
#include <QImage>
#include <QMessageBox>

// Widgets
#include <QCheckBox>
#include <QComboBox>
#include <QDial>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QSlider>
#include <QWidget>

// Layouts
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include "perceive/graphics/image-container.hpp"

class QCheckBox;
class LabeledSlider;

namespace perceive
{
void to_qimage(const ARGBImage& im, QImage& qim);
void to_qimage(const GreyImage& im, QImage& qim);
void to_qimage(const cv::Mat& im, QImage& qim);
void from_qimage(const QImage& qim, ARGBImage& im);

QString to_qstr(string_view s) noexcept;
string from_qstr(const QString& s) noexcept;

string read_asset_file(const string& rsrc_name) noexcept(false);

// These are deprecated, use the values below
void block_signal_set_checked(QCheckBox* cb, bool val) noexcept;
void block_signal_set_checked(QRadioButton* rb, bool val) noexcept;
void block_signal_set_slider(LabeledSlider* ls, real val) noexcept;
void block_signal_set_lineedit(QLineEdit* le, const string_view val) noexcept;
void block_signal_set_combobox(QComboBox* c, const int val) noexcept;

void block_signal_set(QCheckBox* cb, bool val) noexcept;
void block_signal_set(QRadioButton* rb, bool val) noexcept;
void block_signal_set(LabeledSlider* ls, real val) noexcept;
void block_signal_set(QLineEdit* le, const string_view val) noexcept;
void block_signal_set(QComboBox* c, const int val) noexcept;

void widget_to_value(QCheckBox* cb, bool& val) noexcept;
void widget_to_value(QRadioButton* rb, bool& val) noexcept;
void widget_to_value(LabeledSlider* ls, real& val) noexcept;
void widget_to_value(LabeledSlider* ls, float& val) noexcept;
void widget_to_value(LabeledSlider* ls, int& val) noexcept;
void widget_to_value(QLineEdit* le, string& val) noexcept;
void widget_to_value(QComboBox* c, int& val) noexcept;

QWidget* make_hline() noexcept;
QWidget* make_vline() noexcept;

QWidget* make_horizontal_expander(int fixed_width = -1) noexcept;
QWidget* make_vertical_expander(int fixed_height = -1) noexcept;

QLabel* new_label(const string_view text, const char* style_sheet = nullptr);
QLabel* new_header_label(const string_view text);

void set_message_box_width(QMessageBox* msg_box, unsigned w) noexcept;

void for_each_qform_label(QFormLayout* layout,
                          std::function<void(QWidget*)> f) noexcept;

} // namespace perceive
