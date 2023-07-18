// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: message/map/proto/map_clear_area.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_message_2fmap_2fproto_2fmap_5fclear_5farea_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_message_2fmap_2fproto_2fmap_5fclear_5farea_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3014000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3014000 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include "message/map/proto/map_id.pb.h"
#include "message/map/proto/map_geometry.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_message_2fmap_2fproto_2fmap_5fclear_5farea_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_message_2fmap_2fproto_2fmap_5fclear_5farea_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_message_2fmap_2fproto_2fmap_5fclear_5farea_2eproto;
namespace civ {
namespace hdmap {
class ClearArea;
class ClearAreaDefaultTypeInternal;
extern ClearAreaDefaultTypeInternal _ClearArea_default_instance_;
}  // namespace hdmap
}  // namespace civ
PROTOBUF_NAMESPACE_OPEN
template<> ::civ::hdmap::ClearArea* Arena::CreateMaybeMessage<::civ::hdmap::ClearArea>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace civ {
namespace hdmap {

// ===================================================================

class ClearArea PROTOBUF_FINAL :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:civ.hdmap.ClearArea) */ {
 public:
  inline ClearArea() : ClearArea(nullptr) {}
  virtual ~ClearArea();

  ClearArea(const ClearArea& from);
  ClearArea(ClearArea&& from) noexcept
    : ClearArea() {
    *this = ::std::move(from);
  }

  inline ClearArea& operator=(const ClearArea& from) {
    CopyFrom(from);
    return *this;
  }
  inline ClearArea& operator=(ClearArea&& from) noexcept {
    if (GetArena() == from.GetArena()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance);
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const ClearArea& default_instance();

  static inline const ClearArea* internal_default_instance() {
    return reinterpret_cast<const ClearArea*>(
               &_ClearArea_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(ClearArea& a, ClearArea& b) {
    a.Swap(&b);
  }
  inline void Swap(ClearArea* other) {
    if (other == this) return;
    if (GetArena() == other->GetArena()) {
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(ClearArea* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetArena() == other->GetArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ClearArea* New() const final {
    return CreateMaybeMessage<ClearArea>(nullptr);
  }

  ClearArea* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ClearArea>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ClearArea& from);
  void MergeFrom(const ClearArea& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  ::PROTOBUF_NAMESPACE_ID::uint8* _InternalSerialize(
      ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(ClearArea* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "civ.hdmap.ClearArea";
  }
  protected:
  explicit ClearArea(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_message_2fmap_2fproto_2fmap_5fclear_5farea_2eproto);
    return ::descriptor_table_message_2fmap_2fproto_2fmap_5fclear_5farea_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kOverlapIdFieldNumber = 2,
    kIdFieldNumber = 1,
    kPolygonFieldNumber = 3,
  };
  // repeated .civ.hdmap.Id overlap_id = 2;
  int overlap_id_size() const;
  private:
  int _internal_overlap_id_size() const;
  public:
  void clear_overlap_id();
  ::civ::hdmap::Id* mutable_overlap_id(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::civ::hdmap::Id >*
      mutable_overlap_id();
  private:
  const ::civ::hdmap::Id& _internal_overlap_id(int index) const;
  ::civ::hdmap::Id* _internal_add_overlap_id();
  public:
  const ::civ::hdmap::Id& overlap_id(int index) const;
  ::civ::hdmap::Id* add_overlap_id();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::civ::hdmap::Id >&
      overlap_id() const;

  // optional .civ.hdmap.Id id = 1;
  bool has_id() const;
  private:
  bool _internal_has_id() const;
  public:
  void clear_id();
  const ::civ::hdmap::Id& id() const;
  ::civ::hdmap::Id* release_id();
  ::civ::hdmap::Id* mutable_id();
  void set_allocated_id(::civ::hdmap::Id* id);
  private:
  const ::civ::hdmap::Id& _internal_id() const;
  ::civ::hdmap::Id* _internal_mutable_id();
  public:
  void unsafe_arena_set_allocated_id(
      ::civ::hdmap::Id* id);
  ::civ::hdmap::Id* unsafe_arena_release_id();

  // optional .civ.hdmap.Polygon polygon = 3;
  bool has_polygon() const;
  private:
  bool _internal_has_polygon() const;
  public:
  void clear_polygon();
  const ::civ::hdmap::Polygon& polygon() const;
  ::civ::hdmap::Polygon* release_polygon();
  ::civ::hdmap::Polygon* mutable_polygon();
  void set_allocated_polygon(::civ::hdmap::Polygon* polygon);
  private:
  const ::civ::hdmap::Polygon& _internal_polygon() const;
  ::civ::hdmap::Polygon* _internal_mutable_polygon();
  public:
  void unsafe_arena_set_allocated_polygon(
      ::civ::hdmap::Polygon* polygon);
  ::civ::hdmap::Polygon* unsafe_arena_release_polygon();

  // @@protoc_insertion_point(class_scope:civ.hdmap.ClearArea)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::civ::hdmap::Id > overlap_id_;
  ::civ::hdmap::Id* id_;
  ::civ::hdmap::Polygon* polygon_;
  friend struct ::TableStruct_message_2fmap_2fproto_2fmap_5fclear_5farea_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ClearArea

// optional .civ.hdmap.Id id = 1;
inline bool ClearArea::_internal_has_id() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  PROTOBUF_ASSUME(!value || id_ != nullptr);
  return value;
}
inline bool ClearArea::has_id() const {
  return _internal_has_id();
}
inline const ::civ::hdmap::Id& ClearArea::_internal_id() const {
  const ::civ::hdmap::Id* p = id_;
  return p != nullptr ? *p : reinterpret_cast<const ::civ::hdmap::Id&>(
      ::civ::hdmap::_Id_default_instance_);
}
inline const ::civ::hdmap::Id& ClearArea::id() const {
  // @@protoc_insertion_point(field_get:civ.hdmap.ClearArea.id)
  return _internal_id();
}
inline void ClearArea::unsafe_arena_set_allocated_id(
    ::civ::hdmap::Id* id) {
  if (GetArena() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(id_);
  }
  id_ = id;
  if (id) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:civ.hdmap.ClearArea.id)
}
inline ::civ::hdmap::Id* ClearArea::release_id() {
  _has_bits_[0] &= ~0x00000001u;
  ::civ::hdmap::Id* temp = id_;
  id_ = nullptr;
  if (GetArena() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
  return temp;
}
inline ::civ::hdmap::Id* ClearArea::unsafe_arena_release_id() {
  // @@protoc_insertion_point(field_release:civ.hdmap.ClearArea.id)
  _has_bits_[0] &= ~0x00000001u;
  ::civ::hdmap::Id* temp = id_;
  id_ = nullptr;
  return temp;
}
inline ::civ::hdmap::Id* ClearArea::_internal_mutable_id() {
  _has_bits_[0] |= 0x00000001u;
  if (id_ == nullptr) {
    auto* p = CreateMaybeMessage<::civ::hdmap::Id>(GetArena());
    id_ = p;
  }
  return id_;
}
inline ::civ::hdmap::Id* ClearArea::mutable_id() {
  // @@protoc_insertion_point(field_mutable:civ.hdmap.ClearArea.id)
  return _internal_mutable_id();
}
inline void ClearArea::set_allocated_id(::civ::hdmap::Id* id) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArena();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(id_);
  }
  if (id) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
      reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(id)->GetArena();
    if (message_arena != submessage_arena) {
      id = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, id, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  id_ = id;
  // @@protoc_insertion_point(field_set_allocated:civ.hdmap.ClearArea.id)
}

// repeated .civ.hdmap.Id overlap_id = 2;
inline int ClearArea::_internal_overlap_id_size() const {
  return overlap_id_.size();
}
inline int ClearArea::overlap_id_size() const {
  return _internal_overlap_id_size();
}
inline ::civ::hdmap::Id* ClearArea::mutable_overlap_id(int index) {
  // @@protoc_insertion_point(field_mutable:civ.hdmap.ClearArea.overlap_id)
  return overlap_id_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::civ::hdmap::Id >*
ClearArea::mutable_overlap_id() {
  // @@protoc_insertion_point(field_mutable_list:civ.hdmap.ClearArea.overlap_id)
  return &overlap_id_;
}
inline const ::civ::hdmap::Id& ClearArea::_internal_overlap_id(int index) const {
  return overlap_id_.Get(index);
}
inline const ::civ::hdmap::Id& ClearArea::overlap_id(int index) const {
  // @@protoc_insertion_point(field_get:civ.hdmap.ClearArea.overlap_id)
  return _internal_overlap_id(index);
}
inline ::civ::hdmap::Id* ClearArea::_internal_add_overlap_id() {
  return overlap_id_.Add();
}
inline ::civ::hdmap::Id* ClearArea::add_overlap_id() {
  // @@protoc_insertion_point(field_add:civ.hdmap.ClearArea.overlap_id)
  return _internal_add_overlap_id();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::civ::hdmap::Id >&
ClearArea::overlap_id() const {
  // @@protoc_insertion_point(field_list:civ.hdmap.ClearArea.overlap_id)
  return overlap_id_;
}

// optional .civ.hdmap.Polygon polygon = 3;
inline bool ClearArea::_internal_has_polygon() const {
  bool value = (_has_bits_[0] & 0x00000002u) != 0;
  PROTOBUF_ASSUME(!value || polygon_ != nullptr);
  return value;
}
inline bool ClearArea::has_polygon() const {
  return _internal_has_polygon();
}
inline const ::civ::hdmap::Polygon& ClearArea::_internal_polygon() const {
  const ::civ::hdmap::Polygon* p = polygon_;
  return p != nullptr ? *p : reinterpret_cast<const ::civ::hdmap::Polygon&>(
      ::civ::hdmap::_Polygon_default_instance_);
}
inline const ::civ::hdmap::Polygon& ClearArea::polygon() const {
  // @@protoc_insertion_point(field_get:civ.hdmap.ClearArea.polygon)
  return _internal_polygon();
}
inline void ClearArea::unsafe_arena_set_allocated_polygon(
    ::civ::hdmap::Polygon* polygon) {
  if (GetArena() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(polygon_);
  }
  polygon_ = polygon;
  if (polygon) {
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:civ.hdmap.ClearArea.polygon)
}
inline ::civ::hdmap::Polygon* ClearArea::release_polygon() {
  _has_bits_[0] &= ~0x00000002u;
  ::civ::hdmap::Polygon* temp = polygon_;
  polygon_ = nullptr;
  if (GetArena() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
  return temp;
}
inline ::civ::hdmap::Polygon* ClearArea::unsafe_arena_release_polygon() {
  // @@protoc_insertion_point(field_release:civ.hdmap.ClearArea.polygon)
  _has_bits_[0] &= ~0x00000002u;
  ::civ::hdmap::Polygon* temp = polygon_;
  polygon_ = nullptr;
  return temp;
}
inline ::civ::hdmap::Polygon* ClearArea::_internal_mutable_polygon() {
  _has_bits_[0] |= 0x00000002u;
  if (polygon_ == nullptr) {
    auto* p = CreateMaybeMessage<::civ::hdmap::Polygon>(GetArena());
    polygon_ = p;
  }
  return polygon_;
}
inline ::civ::hdmap::Polygon* ClearArea::mutable_polygon() {
  // @@protoc_insertion_point(field_mutable:civ.hdmap.ClearArea.polygon)
  return _internal_mutable_polygon();
}
inline void ClearArea::set_allocated_polygon(::civ::hdmap::Polygon* polygon) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArena();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(polygon_);
  }
  if (polygon) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
      reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(polygon)->GetArena();
    if (message_arena != submessage_arena) {
      polygon = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, polygon, submessage_arena);
    }
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  polygon_ = polygon;
  // @@protoc_insertion_point(field_set_allocated:civ.hdmap.ClearArea.polygon)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace hdmap
}  // namespace civ

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_message_2fmap_2fproto_2fmap_5fclear_5farea_2eproto
