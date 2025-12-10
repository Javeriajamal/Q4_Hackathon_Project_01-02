# Data Model: Physical AI And Humanoid Robotics Book Website

## Entities

### Book Content
- **Type**: Content hierarchy
- **Fields**:
  - id: string (unique identifier for the book)
  - title: string (Physical AI And Humanoid Robotics)
  - chapters: array of Chapter objects (exactly 5 chapters)
  - description: string (book description for homepage)
  - author: string (author name)
  - publicationDate: date (when the book content was published)

### Chapter
- **Type**: Content section
- **Fields**:
  - id: string (unique chapter identifier)
  - title: string (chapter title)
  - number: integer (chapter number 1-5)
  - topics: array of Topic objects (exactly 2 topics per chapter)
  - description: string (brief chapter summary)
  - contentPath: string (path to MDX file)

### Topic
- **Type**: Content subsection
- **Fields**:
  - id: string (unique topic identifier)
  - title: string (topic title)
  - number: integer (topic number 1-2 within chapter)
  - contentPath: string (path to MDX file)
  - estimatedReadingTime: integer (in minutes)

### UserPreference
- **Type**: Client-side settings
- **Fields**:
  - theme: string (light|dark)
  - fontSize: string (small|medium|large)
  - lastReadChapter: string (ID of last chapter read)
  - lastReadPosition: number (scroll position percentage)

### WaitlistEntry
- **Type**: Client-side data storage
- **Fields**:
  - name: string (user's name)
  - email: string (user's email)
  - timestamp: date (when they joined)
  - status: string (active|confirmed|removed)

## Relationships

- Book Content contains 5 Chapters
- Each Chapter contains 2 Topics
- UserPreference stores user settings and reading progress
- WaitlistEntry stores user information for notifications

## Validation Rules

- Book must have exactly 5 chapters
- Each chapter must have exactly 2 topics
- Chapter numbers must be sequential (1-5)
- Topic numbers must be sequential within each chapter (1-2)
- Email in WaitlistEntry must be valid email format
- Theme must be either "light" or "dark"

## State Transitions

- WaitlistEntry: active → confirmed (when user confirms email)
- UserPreference: lastReadPosition updates as user scrolls through content